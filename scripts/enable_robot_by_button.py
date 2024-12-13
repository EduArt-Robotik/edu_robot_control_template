#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from edu_robot.srv import SetMode
from edu_robot.msg import Mode
import mraa

# helper function for getting given mode as string
def get_mode_string(mode : Mode) -> str:
  mode_string = str()

  if mode.mode & Mode.INACTIVE:
    mode_string += 'INACTIVE'
  if mode.mode & Mode.REMOTE_CONTROLLED:
    mode_string += 'REMOTE CONTROLLED|'
  if mode.mode & Mode.AUTONOMOUS:
    mode_string += 'FLEET|'
  if mode.mode & Mode.COLLISION_AVOIDANCE:
    mode_string += 'COLLISION_AVOIDANCE|'
  if mode.mode & Mode.COLLISION_AVOIDANCE_OVERRIDE:
    mode_string += 'COLLISION_AVOIDANCE_OVERRIDE|'
  if mode.mode & Mode.SKID_DRIVE:
    mode_string += 'SKID_DRIVE|'
  if mode.mode & Mode.MECANUM_DRIVE:
    mode_string += 'MECANUM_DRIVE|'
  
  if len(mode_string) > 0:
    mode_string = mode_string[:-1]

  return mode_string


"""This ROS node enables the robot by a button pressure at a GPIO. It shows how a button can be read as input.
This node reads a button as input and triggers an ROS2 service call. The GPIO is controlled by the library MRAA.
"""
class EnableRobotByButton(Node):
  def __init__(self):
    # Calling Constructor of Base Class: Important!
    super().__init__('enable_robot_by_button')

    # Create service client for setting mode.
    self.client_set_mode = self.create_client(SetMode, 'set_mode')

    # GPIO 4
    self.was_pressed = False
    self.enable_button = mraa.Gpio(4)
    # Initialize GPIO pin as input.
    self.enable_button.dir(mraa.DIR_IN)

    # Start timer: the timer calls the bound callback method/function with given time period of 100ms.
    self.timer = self.create_timer(0.1, self.process)

  def process(self):
    button_value = self.enable_button.read()

    if button_value < 0:
      # error occurred during reading of button input value
      self.get_logger().error('error occurred during reading of button input value')
      return
    
    # reading button state: when button is pressed it pulls the signal down to zero voltage.
    is_pressed = button_value == 0

    if self.was_pressed == False and is_pressed == True:
      # detected positive edge --> button was pressed right now
      self.get_logger().info('button was pressed"')

      request = SetMode.Request()
      request.mode.mode = Mode.AUTONOMOUS

      self.client_set_mode.call_async(request)

    self.was_pressed = is_pressed


def main():
  rclpy.init()

  enable_robot_by_button_node = EnableRobotByButton()
  rclpy.spin(enable_robot_by_button_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
  enable_robot_by_button_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
