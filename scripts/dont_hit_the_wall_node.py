#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from edu_robot.srv import SetMode
from edu_robot.msg import Mode

"""ROS node that implements a simple application for the robot Eduard (valid for all hardware variants).
 
This node controls the robot in the scenario and moves forward until it comes close to a wall (or another object),
then it stops in front of the wall. This example shows how to use the basic interfaces of the robot, such as setting
the mode, sending velocity commands (twist) and receiving sensor data (here retrieving distance measurements from
one of the distance sensors).
""" 
class DontHitTheWallNode(Node):
  def __init__(self):
    # Calling Constructor of Base Class: Important!
    super().__init__('dont_hit_the_wall')

    self.close_to_wall = False
    self.robot_is_enabled = False
    self.future_response = None

    # Constructing and Initializing of ROS related Members
    # Subscription    
    self.sub_range = self.create_subscription(
      Range, # topic message type
      'range/front/right/range', # relative topic name of the range sensor
      self.callbackRangeMeasurement, # this callback is called when a new message was received
      QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2
      ) # qos settings that fits to the robot's one
    )

    # Note: without the '/' in front of the topic name it is relative. In this case the node namespace is respected.
    #       For example if the namespace is 'eduard/blue' the complete topic name is 'eduard/blue/range/front/right/range'.
    #       In case the given topic name starts with a '/' the topic name is absolute and the namespace is not respected.
    #       This is valid for publisher and services, too.

    # Publisher
    self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 1) # uses relative topic name, too

    # Service Client
    self.srv_set_mode = self.create_client(SetMode, 'set_mode') # service name is relative

    # Timer
    self.timer_trigger_twist = self.create_timer(0.1, self.sendTwistCommand)


  def callbackRangeMeasurement(self, range_msg):
    self.close_to_wall = range_msg.range < 0.3


  def sendTwistCommand(self):
    # Enabling Robot if not already
    if self.robot_is_enabled is False:
      if self.future_response is None or self.future_response.result() is None:
        set_mode_request = SetMode.Request()
        set_mode_request.mode.mode = Mode.REMOTE_CONTROLLED

        self.future_response = self.srv_set_mode.call_async(set_mode_request)
      else:
        if self.future_response.result().state.mode.mode is Mode.REMOTE_CONTROLLED:
          self.get_logger().info('Robot Eduard is in mode REMOTE_CONTROLLED')
        else:
          self.get_logger().error('Robot Eduard is not in mode REMOTE_CONTROLLED')
  
    # Creating and Initializing Twist Command
    twist_cmd = Twist()

    twist_cmd.linear.x = 0.0 if self.close_to_wall else 0.3
    twist_cmd.linear.y = 0.0
    twist_cmd.linear.z = 0.0

    twist_cmd.angular.x = 0.0
    twist_cmd.angular.y = 0.0
    twist_cmd.angular.z = 0.0

    # Sending Twist Command
    self.pub_twist.publish(twist_cmd)


def main(args=None):
  rclpy.init(args=args)

  node = DontHitTheWallNode()
  rclpy.spin(node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
