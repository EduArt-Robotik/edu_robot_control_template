#include <rclcpp/rclcpp.hpp>
#include <mraa/gpio.hpp>

class ToggleLedByButton : public rclcpp::Node
{
public:
  ToggleLedByButton()
    : rclcpp::Node("toggle_led_by_button")
    , _button(5) // GPIO PIN 5 for button
    , _led(6)    // GPIO PIN 6 for LED
  {
    // Initialize button as input.
    if (mraa::SUCCESS != _button.dir(mraa::DIR_IN)) {
      throw std::runtime_error("Can't set direction for button input.");
    }

    // Initialize LED as output.
    if (mraa::SUCCESS != _led.dir(mraa::DIR_OUT)) {
      throw std::runtime_error("Can't set direction for LED output.");
    }

    // Start timer: calls the callback every 100ms.
    _timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ToggleLedByButton::process, this));
  }

private:
  void process()
  {
    // Read button value.
    const int button_value = _button.read();

    if (button_value < 0) {
      // Error during reading button input value.
      RCLCPP_ERROR(get_logger(), "Error occurred during reading button input value");
      return;
    }

    // Button pressed logic: button pulls signal down to zero voltage when pressed.
    const bool is_pressed = button_value == 0;

    if (!_was_pressed && is_pressed) {
      // Detected positive edge --> button pressed.
      RCLCPP_INFO(get_logger(), "Button was pressed, toggling LED.");

      // Toggle LED state.
      _led_state = !_led_state;
      _led.write(_led_state);
    }

    _was_pressed = is_pressed;
  }

  mraa::Gpio _button;
  mraa::Gpio _led;
  bool _was_pressed = false;
  bool _led_state = false;
  std::shared_ptr<rclcpp::TimerBase> _timer;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToggleLedByButton>());
  rclcpp::shutdown();
  return 0;
}
