#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class Omnibot_Controller : public rclcpp::Node
{
public:
  Omnibot_Controller()
      : Node("Omnibot_Controller")
  {
    cmdVel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Omnibot_Controller::cmdVel_Callback, this, _1));

    forward_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "forward_velocity_controller/commands", 10);

    // Robot-specific parameters
    this->declare_parameter<double>("wheel_radius", 0.0175);
    this->declare_parameter<double>("wheel_offset", 0.038375);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_offset", wheel_base_);
  }

private:
  void cmdVel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    // Forward kinematics to calculate wheel velocities
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    double wheel_fr = (vx - vy - (omega * wheel_base_)) / wheel_radius_;
    double wheel_br = (-vx - vy - (omega * wheel_base_)) / wheel_radius_;
    double wheel_fl = (vx + vy - (omega * wheel_base_)) / wheel_radius_;
    double wheel_bl = (-vx + vy - (omega * wheel_base_)) / wheel_radius_;

    RCLCPP_INFO(
        this->get_logger(),
        "Received cmd_vel: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f | Wheel Speed= %.2f %.2f %.2f %.2f",
        msg->linear.x, msg->linear.y, msg->angular.z, wheel_fr, wheel_br, wheel_fl, wheel_bl);

    // Publish wheel velocities
    std_msgs::msg::Float64MultiArray wheel_vel_msg;
    wheel_vel_msg.data = {wheel_fl, wheel_fr, wheel_bl, wheel_br};
    forward_velocity_publisher->publish(wheel_vel_msg);
  }

  // Parameters
  double wheel_radius_;
  double wheel_base_;
  double wheel_separation_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel_subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_velocity_publisher;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Omnibot_Controller>());
  rclcpp::shutdown();
  return 0;
}