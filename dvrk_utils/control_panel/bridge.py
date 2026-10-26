"""ROS-facing state and commands; only the server's event loop accesses this node."""

import copy
import math
import time

from crtk_msgs.msg import OperatingState, StringStamped
from rclpy.exceptions import InvalidTopicNameException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.validate_full_topic_name import validate_full_topic_name
from std_msgs.msg import Bool, Empty, Float64, String


ARM_NAMES = ('PSM1', 'PSM2', 'PSM3', 'ECM', 'MTML', 'MTMR')


class CommandError(ValueError):
    """A command cannot be accepted in the current state."""


class RosBridge(Node):
    """Keep a browser-ready ROS snapshot and execute the panel's known actions."""

    def __init__(self, console='console', *, context=None):
        """Connect the standard arms and the requested console namespace."""
        super().__init__('dvrk_control_panel_node', context=context)
        self.console = console.strip('/')
        self.state = {
            'console': self.console,
            'arms': {name: {
                'name': name, 'state': 'OFFLINE', 'is_homed': False,
                'is_busy': False, 'tool_type': '', 'seen': False, 'online': False,
                'reset': None,
            } for name in ARM_NAMES},
            'teleops': {}, 'enabled': None, 'scale': None, 'volume': None,
            'available': {}, 'powered': False,
        }
        self.resets = {}
        self._status_subscriptions = []
        self._feedback_subscriptions = {}
        self._last_graph_check = -float('inf')
        self._qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._command_publishers = {}
        prefix = f'/{self.console}/teleop'
        for field, topic, msg_type in (
            ('enabled', f'{prefix}/enabled', Bool),
            ('scale', f'{prefix}/scale', Float64),
            ('volume', '/system/volume', Float64),
        ):
            subscription = self.create_subscription(
                msg_type, topic,
                lambda msg, field=field: self._set_value(field, msg.data), self._qos)
            self._status_subscriptions.append(subscription)
            self._feedback_subscriptions[field] = subscription
        for selected, suffix in ((True, 'selected'), (False, 'unselected')):
            self._status_subscriptions.append(self.create_subscription(
                String, f'{prefix}/{suffix}',
                lambda msg, selected=selected: self._teleop_event(msg.data, selected),
                self._qos))
        for name in ARM_NAMES:
            subscription = self.create_subscription(
                OperatingState, f'/{name}/operating_state',
                lambda msg, name=name: self._arm_state(name, msg), self._qos)
            self._status_subscriptions.append(subscription)
            self._feedback_subscriptions[f'arm:{name}'] = subscription
            self._command_publishers[f'arm:{name}'] = self.create_publisher(
                StringStamped, f'/{name}/state_command', 10)
            if name.startswith('PSM'):
                self._status_subscriptions.append(self.create_subscription(
                    String, f'/{name}/tool_type',
                    lambda msg, name=name: self._tool_type(name, msg.data), self._qos))
        for command, topic, msg_type in (
            ('home', '/system/home', Empty),
            ('power_off', '/system/power_off', Empty),
            ('enable', f'{prefix}/enable', Bool),
            ('select', f'{prefix}/select', String),
            ('unselect', f'{prefix}/unselect', String),
            ('scale', f'{prefix}/set_scale', Float64),
            ('volume', '/system/set_volume', Float64),
        ):
            self._command_publishers[command] = self.create_publisher(msg_type, topic, 10)

    def _set_value(self, field, value):
        if field == 'enabled' or math.isfinite(value):
            self.state[field] = value

    def _arm_state(self, name, msg):
        self.state['arms'][name].update(
            state=msg.state, is_homed=msg.is_homed, is_busy=msg.is_busy,
            seen=True, online=True)

    def _tool_type(self, name, value):
        self.state['arms'][name]['tool_type'] = value

    def _teleop_event(self, name, selected):
        if not name or len(name) > 200:
            return
        try:
            validate_full_topic_name(f'/{name}/current_state')
        except InvalidTopicNameException:
            self.get_logger().warning(f'Ignoring invalid teleoperation name: {name!r}')
            return
        if name not in self.state['teleops']:
            self.state['teleops'][name] = {
                'name': name, 'selected': selected, 'state': 'UNKNOWN', 'online': False,
                'seen': False,
            }
            subscription = self.create_subscription(
                String, f'/{name}/current_state',
                lambda msg, name=name: self._teleop_state(name, msg.data), self._qos)
            self._status_subscriptions.append(subscription)
            self._feedback_subscriptions[f'teleop:{name}'] = subscription
        self.state['teleops'][name]['selected'] = selected

    def _teleop_state(self, name, value):
        self.state['teleops'][name].update(state=value, seen=True, online=True)

    def tick(self, now=None):
        """Refresh ROS availability and advance asynchronous reset sequences."""
        now = time.monotonic() if now is None else now
        if now - self._last_graph_check >= 1.0:
            self._last_graph_check = now
            for name, arm in self.state['arms'].items():
                arm['online'] = arm['seen'] and self._has_publisher(f'arm:{name}')
            for name, teleop in self.state['teleops'].items():
                # Event-only state topics may not emit until the first transition.
                teleop['online'] = self._has_publisher(f'teleop:{name}')
            for field in ('enabled', 'scale', 'volume'):
                self.state['available'][field] = (
                    self._has_publisher(field)
                    and (field == 'enabled' or self.state[field] is not None))
        active = [arm for arm in self.state['arms'].values() if arm['online']]
        self.state['powered'] = bool(active) and all(
            arm['state'] == 'ENABLED' for arm in active)
        self._tick_resets(now)

    def _has_publisher(self, key):
        # Matched endpoints respect both ROS remappings and QoS compatibility.
        return self._feedback_subscriptions[key].get_publisher_count() > 0

    def snapshot(self):
        """Return an independent copy suitable for JSON serialization."""
        return copy.deepcopy(self.state)

    def _publish(self, key, msg):
        publisher = self._command_publishers[key]
        if publisher.get_subscription_count() == 0:
            raise CommandError('No ROS subscriber is connected for this command.')
        publisher.publish(msg)

    def execute(self, command):
        """Validate and publish an action, or reject it without publishing."""
        action = command.get('action')
        if action in ('home', 'power_off'):
            if action == 'home' and self.resets:
                raise CommandError('Wait for the current arm reset before homing the system.')
            self._publish(action, Empty())
            if action == 'power_off':
                for name in list(self.resets):
                    self._finish_reset(name, 'failed', 'Reset cancelled by system power off')
        elif action == 'enable':
            value = command.get('value')
            if type(value) is not bool:
                raise CommandError('Enable requires a boolean value.')
            if not self.state['available'].get('enabled'):
                raise CommandError('Teleoperation status is unavailable.')
            self._publish(action, Bool(data=value))
        elif action in ('scale', 'volume'):
            value = command.get('value')
            minimum, step = (0.2, 0.1) if action == 'scale' else (0.0, 0.05)
            if (type(value) not in (int, float) or not minimum <= value <= 1.0
                    or not math.isfinite(value)):
                raise CommandError(f'{action} must be between {minimum} and 1.0.')
            if not self.state['available'].get(action):
                raise CommandError(f'{action} status is unavailable.')
            value = round(round(value / step) * step, 2)
            self._publish(action, Float64(data=value))
        elif action in ('select', 'unselect'):
            name = command.get('name')
            if not isinstance(name, str) or name not in self.state['teleops']:
                raise CommandError('Unknown teleoperation pair.')
            if not self.state['teleops'][name]['online']:
                raise CommandError('Teleoperation pair is offline.')
            self._publish(action, String(data=name))
        elif action == 'reset':
            name = command.get('name')
            if not isinstance(name, str) or name not in self.state['arms']:
                raise CommandError('Unknown arm.')
            arm = self.state['arms'][name]
            if not arm['online']:
                raise CommandError('Arm is offline.')
            if name in self.resets:
                raise CommandError('Arm reset is already running.')
            self._arm_command(name, 'disable')
            self.resets[name] = {'stage': 'disable', 'started': time.monotonic()}
            arm['reset'] = {'stage': 'disable', 'message': 'Waiting for DISABLED'}
        else:
            raise CommandError('Unknown control action.')

    def _arm_command(self, name, command):
        msg = StringStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.string = command
        self._publish(f'arm:{name}', msg)

    def _finish_reset(self, name, stage, message):
        self.resets.pop(name, None)
        self.state['arms'][name]['reset'] = {'stage': stage, 'message': message}

    def _tick_resets(self, now):
        for name, reset in list(self.resets.items()):
            arm = self.state['arms'][name]
            stage = reset['stage']
            elapsed = now - reset['started']
            if not arm['online']:
                self._finish_reset(name, 'failed', 'Arm disconnected; reset stopped')
                continue
            try:
                if stage == 'disable' and (arm['state'] == 'DISABLED' or elapsed > 3):
                    self._arm_command(name, 'enable')
                    reset.update(stage='enable', started=now)
                    arm['reset'] = {'stage': 'enable', 'message': 'Waiting for ENABLED'}
                elif stage == 'enable':
                    if arm['state'] == 'ENABLED':
                        self._arm_command(name, 'home')
                        reset.update(stage='home', started=now)
                        arm['reset'] = {'stage': 'home', 'message': 'Homing'}
                    elif elapsed > 3:
                        self._finish_reset(name, 'failed', 'Enable timed out')
                elif stage == 'home':
                    if arm['state'] == 'ENABLED' and arm['is_homed'] and not arm['is_busy']:
                        self._finish_reset(name, 'done', 'Reset complete')
                    elif elapsed > 15:
                        self._finish_reset(name, 'failed', 'Homing timed out')
            except CommandError as error:
                self._finish_reset(name, 'failed', str(error))
