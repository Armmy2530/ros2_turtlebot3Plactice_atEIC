#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publisher for servo control
        self.servo_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_joint_position_controller/commands',
            10,
            callback_group=self.callback_group
        )
        
        # Initialize servo position
        self.current_position = 0.0
        self.get_logger().info('Servo test node initialized')

    def move_servo(self, position: float, wait_time: float = 2.0):
        """
        Move servo to specified position and wait
        
        Args:
            position (float): Target position in radians
            wait_time (float): Time to wait after movement in seconds
        """
        try:
            msg = Float64MultiArray(data=[position])
            self.servo_pub.publish(msg)
            self.current_position = position
            self.get_logger().info(f'Moving servo to position: {position}')
            time.sleep(wait_time)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to move servo: {str(e)}')
            return False

    def run_test_sequence(self):
        """Run a predefined test sequence"""
        sequences = [
            (0.0, 2.0),    # Home position
            (1.67, 2.0),   # Maximum position
            (0.0, 2.0),    # Back to home
            (0.8, 2.0),    # Mid position
            (0.0, 2.0)     # Final home position
        ]

        self.get_logger().info('Starting test sequence')
        
        for position, wait_time in sequences:
            if not self.move_servo(position, wait_time):
                self.get_logger().error('Test sequence failed')
                return False
                
        self.get_logger().info('Test sequence completed successfully')
        return True

    def run_interactive_mode(self):
        """Run interactive mode for manual servo control"""
        self.get_logger().info('Starting interactive mode')
        self.get_logger().info('Commands:')
        self.get_logger().info('  position <float>  - Move to specific position')
        self.get_logger().info('  sequence          - Run test sequence')
        self.get_logger().info('  home              - Move to home position')
        self.get_logger().info('  quit              - Exit program')

        while True:
            try:
                command = input('Enter command: ').strip().lower()
                
                if command == 'quit':
                    break
                    
                elif command == 'sequence':
                    self.run_test_sequence()
                    
                elif command == 'home':
                    self.move_servo(0.0)
                    
                elif command.startswith('position'):
                    try:
                        _, position = command.split()
                        position = float(position)
                        if -3.14 <= position <= 3.14:  # Basic range check
                            self.move_servo(position)
                        else:
                            self.get_logger().warning('Position out of range (-3.14 to 3.14)')
                    except ValueError:
                        self.get_logger().error('Invalid position value')
                        
                else:
                    self.get_logger().warning('Unknown command')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create executor and node
    executor = MultiThreadedExecutor()
    node = ServoTestNode()
    executor.add_node(node)

    # Process command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == 'sequence':
            node.run_test_sequence()
        elif sys.argv[1] == 'interactive':
            node.run_interactive_mode()
        elif len(sys.argv) == 3 and sys.argv[1] == 'position':
            try:
                position = float(sys.argv[2])
                node.move_servo(position)
            except ValueError:
                node.get_logger().error('Invalid position value')
    else:
        # Default to interactive mode
        node.run_interactive_mode()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()