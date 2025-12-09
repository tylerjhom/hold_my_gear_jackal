#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com

import time  # Time library

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

'''
Navigates a robot from an initial pose to a goal pose.
'''

JACKAL_NAMESPACE = 'j100_0000'
FOLLOW_GOAL_TOPIC = 'follow_goal'   # topic YOLO node will publish PoseStamped to

def main():
  # Start the ROS 2 Python Client Library
  rclpy.init()

  # Create a node so we can subscribe to things
  node = rclpy.create_node('jackal_follow_nav_client')

  navigator = BasicNavigator(namespace = JACKAL_NAMESPACE)

  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  node.get_logger().info('Waiting for Nav2 to become active...')
  navigator.waitUntilNav2Active(localizer='slam_toolbox')
  node.get_logger().info('Nav2 activated; waiting for follow goals...')

  latest_goal_msg = {'msg': None}  # Stores latest goal_msg
  have_active_task = False
  i = 0   # Feedback counter

  '''
  # # For use in localization # #

  # Set the robot's initial pose if necessary
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = 0.0
  # initial_pose.pose.position.y = 0.0
  # initial_pose.pose.position.z = 0.0
  # initial_pose.pose.orientation.x = 0.0
  # initial_pose.pose.orientation.y = 0.0
  # initial_pose.pose.orientation.z = 0.0
  # initial_pose.pose.orientation.w = 1.0
  # navigator.setInitialPose(initial_pose)

  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')

  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
  '''

  '''
  # # For sending specific locations to robot # #

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.5 # Update when open rviz
  goal_pose.pose.position.y = -0.6 # Update when open rviz
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0

  print(f"DEBUG: goal in script is x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

  # sanity check a valid path exists
  # path = navigator.getPath(initial_pose, goal_pose)

  # Go to the goal pose
  navigator.goToPose(goal_pose)
  '''

  # # For Receiving Goals from Node # #
  def follow_goal_callback(msg: PoseStamped):
     latest_goal_msg['msg'] = msg
     node.get_logger().info(
        f'Received new follow goal (raw): '
        f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

  # Subscribe to goal topic (this also creates the topic!)
  node.create_subscription(PoseStamped, FOLLOW_GOAL_TOPIC, follow_goal_callback, 10)

  
  try:
     while rclpy.ok():
        # Subscription callback
        rclpy.spin_once(node, timeout_sec=0.1)

        # If we get a new goal, run it
        if latest_goal_msg['msg'] is not None:
           msg = latest_goal_msg['msg']
           latest_goal_msg['msg'] = None

           # Cancel previous node
           if have_active_task:
              node.get_logger().info('New goal received, canceling previous Nav2 task.')
              navigator.cancelTask()

           # Assumes that position is already on the map
           goal_pose = PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.header.stamp = navigator.get_clock().now().to_msg()
           goal_pose.pose = msg.pose

           node.get_logger().info(
                    f'Sending Nav2 goal: x={goal_pose.pose.position.x:.2f}, '
                    f'y={goal_pose.pose.position.y:.2f}')
           
           navigator.goToPose(goal_pose)
           have_active_task = True
           i = 0 # Resets feedback counter for new goal task


        # Feedback loop for active tasks
        if have_active_task and not navigator.isTaskComplete():
           i += 1
           feedback = navigator.getFeedback()

           if feedback and i % 5 == 0:
              print('Distance remaining: ' + '{:.2f}'.format(
              feedback.distance_remaining) + ' meters.')

           if feedback: # Extra 'if' in case feedback is None

            # If we get close enough, we can move on
            if feedback.distance_remaining < 0.5:
                node.get_logger.info('Good enough for me!')
                navigator.cancelTask()
                have_active_task = False
                continue
        
            # Navigation timeout
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):  # Changed max duration from 600sec to 300sec
                node.get_logger.info('Took too long, canceling task.')
                navigator.cancelTask()
                have_active_task = False
                continue

            # # Adjust nav position if not working
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
            #     node.get_logger.info('Taking a while, adjusting the goal position...')
            #     preempt_goal = PoseStamped()
            #     preempt_goal.header.frame_id = 'map'
            #     preempt_goal.header.stamp = navigator.get_clock().now().to_msg()
            #     preempt_goal.pose.position.x = -0.3
            #     preempt_goal.pose.position.y = 0.0
            #     preempt_goal.pose.position.z = 0.0
            #     preempt_goal.pose.orientation.w = 1.0
            #     navigator.goToPose(preempt_goal)

        if have_active_task and navigator.isTaskComplete():
           result = navigator.getResult()
           if result == TaskResult.SUCCEEDED:
              print('Goal succeeded!')
           elif result == TaskResult.CANCELED:
              print('Goal was canceled!')
           elif result == TaskResult.FAILED:
              print('Goal failed!')
           else:
              print('Goal has an invalid return status!')
           have_active_task = False
  
  except KeyboardInterrupt:
     node.get_logger.info('Ctrl+C detected, canceling navigation task...')
     navigator.cancelTask()




  # Shut down the ROS 2 Navigation Stack
  # navigator.lifecycleShutdown() # This line tries to shut down the nav2 and slam launch files, but I'm running those through clearpath so we don't want to do that
  
  node.get_logger().info('Shutting down follow-nav client.')
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

