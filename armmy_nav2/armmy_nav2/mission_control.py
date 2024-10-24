import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import math
import time

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # Flag to track if AMCL has localized
        # self.amcl_active = False

        # Subscribe to /amcl_pose to monitor if AMCL is active
        # self.amcl_subscriber = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/amcl_pose',
        #     self.amcl_pose_callback,
        #     10
        # )

        # Create BasicNavigator instance
        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is activated')

        # Publisher for servo control
        self.servo_pub = self.create_publisher(Float64MultiArray, '/arm_joint_position_controller/commands', 10)

        self.init_pose = [-0.035,1.577,0.0]
        # Define goal poses (x, y, yaw)
        self.goal_poses = [ 
            [3.80,-0.03,0.0],  # goal_pose1
            # [2.0, 0.0, 0.0],  # goal_pose2
            # [3.0, 0.0, 0.0]   # goal_pose3
        ]

        self.set_initial_pose(*self.init_pose)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback to check AMCL localization status."""
        # Debug: Log or print the init_pose to ensure it is being accessed
        self.get_logger().info(f'Initial pose: {self.init_pose}')
        
        # Check the covariance of the pose, which decreases when localization is good
        covariance_threshold = 0.05  # Example threshold for checking localization
        covariance_sum = sum(msg.pose.covariance)

        if covariance_sum < covariance_threshold:
            self.amcl_active = True
            self.get_logger().info('AMCL localization complete, skipping initial pose setting.')
        else:
            self.amcl_active = False
            self.get_logger().info('AMCL localization not yet complete. Setting initial pose.')
            self.set_initial_pose(*self.init_pose)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        q = [0]*4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    
    def set_initial_pose(self,x=-0.035,y=1.577,yaw=0.0):
        initial_pose = PoseStamped()

        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = -x
        initial_pose.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, yaw)

        initial_pose.pose.orientation.w = q[0]
        initial_pose.pose.orientation.x = q[1]
        initial_pose.pose.orientation.y = q[2]
        initial_pose.pose.orientation.z = q[3]

        self.navigator.setInitialPose(initial_pose)

    def send_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, yaw)

        goal_pose.pose.orientation.w = q[0]
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]

        self.get_logger().info(f'Sending goal: {goal_pose.pose}')
        self.navigator.goToPose(goal_pose)

        # Wait until the goal is reached
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Current robot position: {feedback.current_pose}')

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully.')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled.')
        else:
            self.get_logger().info('Failed to reach goal.')

    def control_servo(self):
        self.servo_pub.publish(Float64MultiArray(data=[0.0]))
        self.get_logger().info('Servo set to 0')
        time.sleep(2.0)

        self.servo_pub.publish(Float64MultiArray(data=[1.67]))
        self.get_logger().info('Servo set to 1.67')
        time.sleep(2.0)

        self.servo_pub.publish(Float64MultiArray(data=[0.0]))
        self.get_logger().info('Servo reset to 0')
        time.sleep(2.0)

    def goAndDo(self, goal, do_function):
            """Navigate to the goal and execute the provided function."""
            self.get_logger().info(f'Navigating to: {goal}')
            self.send_goal(*goal)
            
            # Execute the function passed in
            do_function()

    def execute_mission(self):
        self.goAndDo(self.goal_poses[0], self.control_servo)
        # self.goAndDo(self.goals[1], self.control_servo)
        # self.goAndDo(self.goals[2], self.control_servo)

    # def execute_mission(self):
    #     for goal in self.goal_poses:
    #         self.get_logger().info(f'Navigating to: {goal}')
    #         self.send_goal(goal['x'], goal['y'], goal['yaw'])

    #         # Control the servo at each goal pose
    #         self.control_servo()

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    # Execute the mission
    node.execute_mission()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
