#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define WHEELS 2
#define WHEEL_Radian 0.033
#define WHEEL_Offset 0.080

#define USE_SIM_TIME_ true
#define ODOM_Method 1 // 0:Euler 1:RUNGE_KUTTA

using std::placeholders::_1;

class Diffdrive_Controller : public rclcpp::Node
{
public:
  Diffdrive_Controller()
      : Node("Diffdrive_Controller")
  {
    cmdVel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Diffdrive_Controller::cmdVel_Callback, this, _1));

    forward_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "forward_velocity_controller/commands", 10);

    joint_state_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Diffdrive_Controller::joint_state_Callback, this, _1));

    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom", 10);
    
    this->set_parameter(rclcpp::Parameter("use_sim_time", USE_SIM_TIME_));

    this->declare_parameter<bool>("debug_cmd_vel", false);
    this->declare_parameter<bool>("debug_odom", false);

    this->get_parameter("debug_cmd_vel", cmd_vel_debug);
    this->get_parameter("debug_odom", odometry_debug);

    // Robot-specific parameters
    this->declare_parameter<double>("wheel_radius", WHEEL_Radian);
    this->declare_parameter<double>("wheel_offset", WHEEL_Offset);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_offset", wheel_base_);


    //initialize Pose
    this->declare_parameter<double>("init_x", 0.00);
    this->declare_parameter<double>("init_y", 0.00);
    this->declare_parameter<double>("init_theta", 0.00);

    this->get_parameter("init_x", x);
    this->get_parameter("init_y", y);
    this->get_parameter("init_theta", theta);

    prev_time = this->get_clock()->now();
    curr_time = this->get_clock()->now();

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
        this->get_logger(),
        "OmniController stated!!! Wheel radian: %f Offset: %f",wheel_radius_, wheel_base_
        );
  }

private:
  void cmdVel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    double vx = msg->linear.x;
    double omega = msg->angular.z;

    double wheel_l = (vx - (omega * wheel_base_)) / wheel_radius_;
    double wheel_r = (vx + (omega * wheel_base_)) / wheel_radius_;

    if(cmd_vel_debug){
      RCLCPP_INFO(
          this->get_logger(),
          "\nReceived cmd_vel:%.2f, %.2f, %.2f \nWheel Speed= %.2f %.2f",
          msg->linear.x, msg->linear.y, msg->angular.z, wheel_l, wheel_r
        );
    }

    // Publish wheel velocities
    std_msgs::msg::Float64MultiArray wheel_vel_msg;
    wheel_vel_msg.data = {wheel_l, wheel_r};
    forward_velocity_publisher->publish(wheel_vel_msg);
  }

  void joint_state_Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    curr_time = this->get_clock()->now();
    auto dt = (curr_time - prev_time).seconds();
    
    // Prevent division by zero or very small time differences
    if (abs(dt) <= 0.000001) {
        // RCLCPP_WARN(this->get_logger(), "Skipping dt: %.6f curr_time: %.6f prev_time: %.6f", dt, curr_time.seconds(), prev_time.seconds());
        return;
    }

    std::vector<double> wheel_position = msg->position;
    double delta_pos[4]; // 0: BL 1:BR 2:FR 3:FL

    for (int i = 0; i < WHEELS; i++) {
        curr_pos[i] = wheel_position[i];
        delta_pos[i] = curr_pos[i] - prev_pos[i];

        // ticks to wheel angular velocity, radians/second (more accurate)
        // compute wheel angular velocity with number of ticks on motor encoder,
        // divided by total number of enc * time passed * gear ratio  //360 * 4096 * T;
        wheel_omega[i] = delta_pos[i] / dt; // rad / s
        prev_pos[i] = curr_pos[i];
    }

    /* forward kinematic */
    double v_y   = (wheel_radius_ / 4) * 1.414213562 * (-wheel_omega[2] + wheel_omega[1] - wheel_omega[3] + wheel_omega[0]);
    double v_x   = (wheel_radius_ / 4) * 1.414213562 * (-wheel_omega[2] - wheel_omega[1] + wheel_omega[3] + wheel_omega[0]);
    double omega = (wheel_radius_ / 4) * (-wheel_omega[2] - wheel_omega[1] - wheel_omega[3] - wheel_omega[0]) * (1 / (wheel_base_));

    /* odometry */
    double delta_x = v_x * dt;
    double delta_y = v_y * dt;
    double delta_theta = omega * dt;

    theta += delta_theta;

    switch(ODOM_Method) {
        case 0: 
            // Euler
            x += delta_x * std::cos(theta) - delta_y * std::sin(theta);
            y += delta_x * std::sin(theta) + delta_y * std::cos(theta);
            break;

        case 1:
            // Runge-Kutta
            x += delta_x * std::cos(theta + omega * dt / 2) - delta_y * std::sin(theta + omega * dt / 2);
            y += delta_x * std::sin(theta + omega * dt / 2) + delta_y * std::cos(theta + omega * dt / 2);
            break;
    }

    // Publish Odometry
    // std_msgs::msg::Float64MultiArray wheel_vel_msg;
    // wheel_vel_msg.data = {wheel_fr, wheel_br, wheel_fl, wheel_bl};
    // forward_velocity_publisher->publish(wheel_vel_msg);

    // odometry position published into topic '/odom'
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = curr_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    // My method is x+ -> Right   y+ -> up
    // But ROS is   x+ -> Up     y+ -> Left
    // Very Good!!! my head is hurt

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Assign velocities to twist
    odom_msg.twist.twist.linear.x = v_x; // Linear velocity x
    odom_msg.twist.twist.linear.y = v_y; // Linear velocity y
    odom_msg.twist.twist.linear.z = 0.0;               // No linear motion in z
    odom_msg.twist.twist.angular.x = 0.0;              // No angular motion in x
    odom_msg.twist.twist.angular.y = 0.0;              // No angular motion in y
    odom_msg.twist.twist.angular.z = omega; // Angular velocity z

    odometry_publisher->publish(odom_msg);

    // Create a transform message
    geometry_msgs::msg::TransformStamped transform_stamped;
    // Set up the transform values
    transform_stamped.header.stamp = curr_time;  // Timestamp
    transform_stamped.header.frame_id = "odom";  // Parent frame
    transform_stamped.child_frame_id = "base_footprint";  // Child frame

    // Set translation (x, y, z)
    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = 0.0;

    // Set rotation using the quaternion
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // Publish the transform
    tf_broadcaster->sendTransform(transform_stamped);

    // Debug Message
    if(odometry_debug){
      // RCLCPP_INFO(
      //   this->get_logger(),
      //   "dt: %.6f d_pos: %.6f omega: %.6f", dt, delta_pos[0], wheel_omega[0]
      // );
      // RCLCPP_INFO(
      //   this->get_logger(),
      //   "Wheel position: %.2f %.2f %.2f %.2f", wheel_position[0], wheel_position[1], wheel_position[2], wheel_position[3]
      // );
      RCLCPP_INFO(
        this->get_logger(),
        "Wheel Angular Velocity: %.6f %.6f %.6f %.6f", wheel_omega[0], wheel_omega[1], wheel_omega[2], wheel_omega[3]
      );
      RCLCPP_INFO(
        this->get_logger(),
        "vx: %.6f vy: %.6f omega: %.6f", v_x, v_y, omega
      );
      RCLCPP_INFO(  
        this->get_logger(),
        "x: %.6f y: %.6f theta: %.6f", x, y, theta
      );
    }

    prev_time = curr_time;
  }

  // Parameters
  double wheel_radius_;
  double wheel_base_;
  double wheel_separation_;

  double wheel_omega[4]; // Array for wheel angular velocities
  double curr_pos[4];    // Current positions of the wheels
  double prev_pos[4];    // Previous positions of the wheels

  // Odometery
  double x,y,theta; 

  bool cmd_vel_debug, odometry_debug; //Debug Parameter

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time prev_time;
  rclcpp::Time curr_time;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel_subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_velocity_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
  };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Diffdrive_Controller>());
  rclcpp::shutdown();
  return 0;
}