#include <string>

//ros2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

#include "hunter_base/hunter_messenger.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("hunter_base");
  auto node = rclcpp::Node::make_shared("")
  auto private_node = rclcpp::Node::make_share("~")

  // instantiate a robot object
  HunterBase robot;
  HunterROSMessenger messenger(&robot, &node);

  // fetch parameters before connecting to robot
  std::string port_name;
  /* private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50); */

  private_node.declare_parameter("port_name", "can0");
  private_node.set_parameter(rclcpp::Parameter("port_name", port_name));
  private_node.declare_parameter("odom_frame", "odom");
  private_node.set_parameter(rclcpp::Parameter("odom_frame", messenger.odom_frame_));
  private_node.declare_parameter("base_frame", "base_link");
  private_node.set_parameter(rclcpp::Parameter("base_frame", messenger.base_frame_));
  private_node.declare_parameter("simulated_robot", false);
  private_node.set_parameter(rclcpp::Parameter("simulated_robot", messenger.simulated_robot_));
  private_node.declare_parameter("control_rate", 50);
  private_node.set_parameter(rclcpp::Parameter("control_rate", messenger.sim_control_rate_));

  // connect to robot and setup ROS subscription
  if (port_name.find("can") != std::string::npos) {
    robot.Connect(port_name);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using CAN bus to talk with the robot: %s\n", port_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Only CAN bus interface is supported for now");
    return -1;
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  rclcpp::Rate  rate_50hz(50);  // 50Hz
  int cnt = 0;
  while (rclcpp::ok()) {
    messenger.PublishStateToROS();
    rclcpp::spin_some();
    rate_50hz.sleep();
    // if(++cnt == 390) break;
  }

  return 0;
}