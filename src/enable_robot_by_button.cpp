//################################################################
//# Copyright (C) 2024, EduArt Robotik GmbH, All rights reserved.#
//# Further information can be found in the LICENSE file.        #
//################################################################

#include <rclcpp/rclcpp.hpp>

#include <edu_robot/srv/set_mode.hpp>

#include <mraa/gpio.hpp>

using namespace std::chrono_literals;

/**
 * \brief This ROS node enables the robot by a button pressure at a GPIO. It shows how a button can be read as input.
 *
 * This node reads a button as input and triggers an ROS2 service call. The GPIO is controlled by the library MRAA.
 */
class EnableRobotByButton : public rclcpp::Node
{
public:
  EnableRobotByButton()
    : rclcpp::Node("enable_robot_by_button")
    , _enable_button(4) // GPIO PIN 4
  {
    // Create service client for setting mode.
    _client_set_mode = create_client<edu_robot::srv::SetMode>("set_mode");

    // Initialize GPIO pin as input.
    if (mraa::SUCCESS != _enable_button.dir(mraa::DIR_IN)) {
      throw std::runtime_error("Can't set direction for button input.");
    }

    // Start timer: the timer calls the bound callback method/function with given time period of 100ms.
    _timer = create_timer(100ms, std::bind(&EnableRobotByButton::process, this));
  }

private:
  void process()
  {

  }

  std::shared_ptr<rclcpp::Client<edu_robot::srv::SetMode>> _client_set_mode;
  std::shared_ptr<rclcpp::TimerBase> _timer;
  mraa::Gpio _enable_button;
  bool _was_pressed = false;
};

int main(int argc, char** argv)
{
  // Initializing ROS
  rclcpp::init(argc, argv);
  // Instantiating
  rclcpp::spin(std::make_shared<EnableRobotByButton>());
  // Shuting down ROS after the ROS node exited
  rclcpp::shutdown();
}