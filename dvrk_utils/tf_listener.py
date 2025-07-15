import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class TransformRetriever(Node):
    def __init__(self):
        super().__init__('transform_retriever')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

    def get_transform(self,
                      parent_frame,
                      child_frame) -> TransformStamped:
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent_frame,
                    child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
                )
                self.tf_buffer.clear()

                self.get_logger().debug(f"Transform received: {transform}")
                return transform
            except Exception as e:
                self.get_logger().info(f"Waiting for transform: {e}")

    def destroy_node(self):
        if hasattr(self.tf_listener, 'executor') and self.tf_listener.executor is not None:
            self.tf_listener.executor.shutdown()

        return super().destroy_node()


def main():
    import sys
    from dvrk_utils.utils import stamped_to_se3

    rclpy.init()
    if len(sys.argv) < 3:
        print("Usage: python tf_listener.py <parent_frame> <child_frame>")
        return

    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]

    node = TransformRetriever()
    try:
        transform = node.get_transform(parent_frame, child_frame)
        transform = stamped_to_se3(transform)
        node.get_logger().info(
            f"Transform from {parent_frame} to {child_frame}: \n{transform}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
