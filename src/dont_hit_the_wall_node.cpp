//################################################################
//# Copyright (C) 2023, EduArt Robotik GmbH, All rights reserved.#
//# Further information can be found in the LICENSE file.        #
//################################################################

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <edu_robot/srv/set_mode.hpp>

using namespace std::chrono_literals;

/**
 * \brief ROS node that implements a simple application for the robot Eduard (valid for all hardware variants).
 *
 * This node controls the robot in the scenario and moves forward until it comes close to a wall (or another object),
 * then it stops in front of the wall. This example shows how to use the basic interfaces of the robot, such as setting
 * the mode, sending velocity commands (twist) and receiving sensor data (here retrieving distance measurements from
 * one of the distance sensors).
 */
class DontHitTheWallNode : public rclcpp::Node
{
public:
  DontHitTheWallNode()
    : rclcpp::Node("dont_hit_the_wall")
  {
    // Constructing and Initializing of ROS related Members
    // Subscription
    _sub_range = create_subscription<sensor_msgs::msg::Range>(
      "range/front/right/range", // relative topic name of the range sensor
      rclcpp::QoS(2).best_effort().durability_volatile(), // qos settings that fits to the robot's one
      std::bind(&DontHitTheWallNode::callbackRangeMeasurement, this, std::placeholders::_1) // this callback is called when a new message was received
    );

    // Note: without the '/' in front of the topic name it is relative. In this case the node namespace is respected.
    //       For example if the namespace is 'eduard/blue' the complete topic name is 'eduard/blue/range/front/right/range'.
    //       In case the given topic name starts with a '/' the topic name is absolute and the namespace is not respected.
    //       This is valid for publisher and services, too.

    // Publisher
    _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", // relative topic name of the twist input of the robot
      rclcpp::QoS(2).best_effort().durability_volatile() // qos settings that fits to the robot's one
    );

    // Service Client
    _srv_set_mode = create_client<edu_robot::srv::SetMode>("set_mode"); // service name is relative

    // Timer
    _timer_trigger_twist = create_wall_timer(100ms, std::bind(&DontHitTheWallNode::sendTwistCommand, this));
  }

private:
  void callbackRangeMeasurement(std::shared_ptr<const sensor_msgs::msg::Range> range_msg)
  {
    // If the robot is closer than 0.3m stop driving forward.
    _close_to_wall = range_msg->range < 0.3;
  }

  void sendTwistCommand()
  {
    // Enabling Robot if not already
    using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest; // for better readability
    
    if (_robot_is_enabled == false) {
      auto set_mode_request = std::make_shared<edu_robot::srv::SetMode::Request>(); // create set mode request
      set_mode_request->mode.mode = edu_robot::msg::Mode::REMOTE_CONTROLLED;
      RCLCPP_INFO(get_logger(), "sending request");
      _srv_set_mode->async_send_request(
        set_mode_request, // sending request
        [this](ResponseFuture future){ // defining lambda callback for handling response
          const auto response = future.get();

          if (response.second->state.mode.mode & edu_robot::msg::Mode::REMOTE_CONTROLLED) {
            RCLCPP_INFO(get_logger(), "Robot Eduard is in mode REMOTE_CONTROLLED");
            _robot_is_enabled = true;
          }
          else {
            RCLCPP_ERROR(get_logger(), "Robot Eduard is not in mode REMOTE_CONTROLLED");
          }
        }
      );
    }

    // Creating and Initializing Twist Command
    geometry_msgs::msg::Twist twist_cmd;

    twist_cmd.linear.x = _close_to_wall ? 0.0 : 0.3; // If the robot is close to wall set zero otherwise 0.3 m/s.
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;

    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;

    // Sending Twist Command
    _pub_twist->publish(twist_cmd);
  }

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Range>> _sub_range; //> subscription to a range sensor
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;  //> publisher for sending twist commands
  std::shared_ptr<rclcpp::Client<edu_robot::srv::SetMode>> _srv_set_mode;    //> service client for setting robot's mode
  std::shared_ptr<rclcpp::TimerBase> _timer_trigger_twist;                   //> timer used to send frequent twist commands

  bool _close_to_wall = false; //> indicates if the robot is close to the wall
  bool _robot_is_enabled = false; //> indicates if the robot is in mode REMOTE_CONTROLLED
};

int main(int argc, char** argv)
{
  // Initializing ROS
  rclcpp::init(argc, argv);
  // Instantiating Don't hit the wall ROS Node
  rclcpp::spin(std::make_shared<DontHitTheWallNode>());
  // Shuting down ROS after the ROS node exited
  rclcpp::shutdown();    
}