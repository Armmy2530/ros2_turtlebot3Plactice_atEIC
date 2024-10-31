import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Adjust the import based on the message type

class TopicBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # Log a message when the node starts
        self.get_logger().info("Odom Bridge Node has started.")
        
        self.subscription = self.create_subscription(
            Odometry,              # Change this to the correct message type
            '/diff_cont/odom',    # Input topic
            self.listener_callback,
            10)                   # QoS profile
        
        self.publisher = self.create_publisher(
            Odometry,             # Change this to the correct message type
            '/odom',              # Output topic
            10)                   # QoS profile

    def listener_callback(self, msg):
        self.publisher.publish(msg)  # Publish the received message to the new topic

def main(args=None):
    rclpy.init(args=args)
    bridge_node = TopicBridge()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
