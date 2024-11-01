import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from time import time

class OdometryDelayMeasurer(Node):
    def __init__(self):
        super().__init__('odometry_delay_measurer')
        
        # Initialize last message timestamps and delay storage
        self.last_diff_cont_odom_time = None
        self.last_odom_time = None
        self.delays = []
        self.start_time = time()

        # Subscribers for each topic
        self.diff_cont_odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.diff_cont_odom_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def diff_cont_odom_callback(self, msg):
        # Store the timestamp of the latest message from /diff_cont/odom
        self.last_diff_cont_odom_time = msg.header.stamp
        self.calculate_delay()

    def odom_callback(self, msg):
        # Store the timestamp of the latest message from /odom
        self.last_odom_time = msg.header.stamp
        self.calculate_delay()

    def calculate_delay(self):
        # Ensure both timestamps are available
        if self.last_diff_cont_odom_time and self.last_odom_time:
            # Calculate delay in milliseconds
            delay_ms = abs((self.last_diff_cont_odom_time.sec + self.last_diff_cont_odom_time.nanosec * 1e-9) -
                           (self.last_odom_time.sec + self.last_odom_time.nanosec * 1e-9)) * 1000
            self.delays.append(delay_ms)

            # Check if 1 second has passed to calculate and display the average delay
            current_time = time()
            if current_time - self.start_time >= 1.0:
                if self.delays:
                    avg_delay = sum(self.delays) / len(self.delays)
                    self.get_logger().info(f"Average delay between /diff_cont/odom and /odom: {avg_delay:.2f} ms")
                else:
                    self.get_logger().info("No delay data available for averaging.")
                
                # Reset for the next 1-second interval
                self.delays.clear()
                self.start_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryDelayMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
