#! /usr/bin/env python3

import rclpy                                                                    # For ROS2
import os                                                                       # For environment variables
from geometry_msgs.msg import PoseStamped                                       # For Goal poses 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult    # For NAV2 Simple Commander API 
from edu_robot.srv import SetMode                                               # To activate robot


# Global Variables
namespace = os.environ['EDU_ROBOT_NAMESPACE']       # Get the environment variable
frame_id = namespace + '/map'                       # Frame in which the goal poses are being published


def main():
    # Initialize ROS2
    rclpy.init()

    # Create a NAV2 Simple Commander Navigator
    navigator = BasicNavigator()    

    # Debug Output    
    logger = navigator.get_logger()
    logger.info('Robot Namespace: ' + namespace)

    # Check if NAV2 is activated
    navigator.waitUntilNav2Active()

    # Create poses
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = frame_id
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -2.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 0.0
    goal_pose1.pose.orientation.z = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = frame_id
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 10.0
    goal_pose2.pose.position.y = -6.5
    goal_pose2.pose.orientation.w = 1.0
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = frame_id
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.0
    goal_pose3.pose.position.y = 0.0
    goal_pose3.pose.orientation.w = 1.0
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

    # Set goal poses for NAV2
    navigator.followWaypoints(goal_poses)

    # Set edu_robot to "autonomous mode"
    success = activateRobot(navigator)
    if not success:
        exit(1)

    # Check if Tasks are complete + info output
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            logger.info(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses)), 
            )

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        logger.info('Reached all goals!')
    elif result == TaskResult.FAILED:
        logger.error('Something went wrong, goal failed!')
    else:
        logger.error('Invalid return status!')

    # End program 
    exit(0)


# This method encapsulates the activation of the robot
# To allow the robot to drive, its mode must be set to remote controlled or autonomous.
# Since the robot drives via NAV2, it must be set to "autonomous". 
# This is done via a service call.
def activateRobot(node) -> bool:
    client = node.create_client(SetMode, '/' + namespace + '/set_mode')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for '+ namespace + '/set_mode service...')

    # Create service request object
    request = SetMode.Request()
    request.mode.mode               = 4     # 0=Unknown, 1=Inactive, 2=Remote Controlled, 4=Autonomous, 8=Charging 
    request.mode.drive_kinematic    = 2     # 1=Skid, 2=Mecanum
    request.mode.feature_mode       = 1     # 0=None, 1=Collision Avoidance, 2=Collision Avoidance Override
    request.disable_feature         = 0     # Inactive

    # Send request and wait for result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # Check result
    if future.result() is not None:
        node.get_logger().info('Service call successful - Autonomous mode activated')
        return True
    else:
        node.get_logger().error(f'Service response: {future.result()}')
        return False


# Entry point
if __name__ == '__main__':
    main()