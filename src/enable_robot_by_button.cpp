//################################################################
//# Copyright (C) 2024, EduArt Robotik GmbH, All rights reserved.#
//# Further information can be found in the LICENSE file.        #
//################################################################

#include <rclcpp/rclcpp.hpp>

#include <edu_robot/srv/set_mode.hpp>

#include <mraa/gpio.hpp>

using namespace std::chrono_literals;

// helper function for getting given mode as string
std::string get_mode_string(const edu_robot::msg::Mode mode)
{
  std::string mode_string;

  if (mode.mode & edu_robot::msg::Mode::INACTIVE) {
    mode_string += "INACTIVE|";
  }
  if (mode.mode & edu_robot::msg::Mode::REMOTE_CONTROLLED) {
    mode_string += "REMOTE CONTROLLED|";
  }
  if (mode.mode & edu_robot::msg::Mode::AUTONOMOUS) {
    mode_string += "FLEET|";
  }
  if (mode.feature_mode & edu_robot::msg::Mode::COLLISION_AVOIDANCE) {
    mode_string += "COLLISION_AVOIDANCE|";
  }
  if (mode.feature_mode & edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE) {
    mode_string += "COLLISION_AVOIDANCE_OVERRIDE|";
  }  
  if (mode.drive_kinematic & edu_robot::msg::Mode::SKID_DRIVE) {
    mode_string += "SKID_DRIVE|";
  }
  if (mode.drive_kinematic & edu_robot::msg::Mode::MECANUM_DRIVE) {
    mode_string += "MECANUM_DRIVE|";
  }

  if (mode_string.empty() == false) {
    mode_string.pop_back();
  }

  return mode_string;
}

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
    // alias for service response future type (just for better reading)
    using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest;

    // is called by the timer every 100ms
    const int button_value = _enable_button.read();

    if (button_value < 0) {
      // error occurred during reading of button input value
      RCLCPP_ERROR(get_logger(), "error occurred during reading of button input value");
      return;
    }

    // reading button state: when button is pressed it pulls the signal down to zero voltage.
    const bool is_pressed = button_value == 0;

    if (_was_pressed == false && is_pressed == true) {
      // detected positive edge --> button was pressed right now
      RCLCPP_INFO(get_logger(), "button was pressed");

      auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
      // set robot mode to autonomous
      request->mode.mode = edu_robot::msg::Mode::AUTONOMOUS;

      // call service async and process response in a lambda function
      _client_set_mode->async_send_request(request, [logger = get_logger()](ResponseFuture future){
        const auto response = future.get();

        if ((response.second->state.mode.mode & response.first->mode.mode) == false) {
          RCLCPP_ERROR_STREAM(logger, "Can't disable robot! Robot is in mode = " << get_mode_string(response.second->state.mode));
          return;
        }

        RCLCPP_INFO(logger, "Set mode AUTONOMOUS successfully.");
        RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response.second->state.mode).c_str());      
      });
    }

    _was_pressed = is_pressed;
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