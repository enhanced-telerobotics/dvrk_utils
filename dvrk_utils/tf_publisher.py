from pathlib import Path
import numpy as np
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class StaticTransformPublisher(Node):
    def __init__(self, transform_file: str) -> None:
        super().__init__("static_transform_publisher")

        # Allow the publish rate to be overridden from a launch/CLI argument.
        self.declare_parameter("publish_rate", 100.0)  # Default rate is 100 Hz

        file_path = Path(transform_file).expanduser()

        if not file_path.is_file():
            self.get_logger().error(f"Transform file not found: {file_path}")
            rclpy.shutdown()
            return

        # Load and validate the matrix.
        matrix = np.load(file_path)
        if matrix.shape != (4, 4):
            self.get_logger().error(
                f"{file_path} must contain a 4x4 homogeneous matrix; got {matrix.shape}"
            )
            rclpy.shutdown()
            return

        # <parent>_T_<child>.npy  →  parent_frame, child_frame
        try:
            parent_frame, child_frame = file_path.stem.split("_T_", maxsplit=1)
        except ValueError:
            self.get_logger().error(
                "File name must follow <parent>_T_<child>.npy, e.g. ECM_base_T_Board.npy"
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Publishing static transform: {parent_frame} ➜ {child_frame}"
        )

        self._broadcaster = StaticTransformBroadcaster(self)
        self._transform = self._matrix_to_transform(matrix, parent_frame, child_frame)
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

    @staticmethod
    def _matrix_to_transform(
        mat: np.ndarray, parent: str, child: str
    ) -> TransformStamped:
        """Convert a 4x4 matrix to geometry_msgs/TransformStamped."""
        msg = TransformStamped()
        msg.header.stamp = rclpy.time.Time().to_msg()  # time=0 is fine for static TF
        msg.header.frame_id = parent
        msg.child_frame_id = child

        # Translation
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = mat[
            :3, 3
        ]

        # Rotation
        q = R.from_matrix(mat[:3, :3]).as_quat()  # (x, y, z, w)
        (
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ) = q

        return msg

    def publish_transform(self):
        """Continuously publish the transform at the specified rate."""
        rate = self.create_rate(self._publish_rate)
        while rclpy.ok():
            self._broadcaster.sendTransform(self._transform)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: python tf_publisher.py <transform_file>")
        return

    transform_file = sys.argv[1]
    node = StaticTransformPublisher(transform_file)

    try:
        node.publish_transform()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
