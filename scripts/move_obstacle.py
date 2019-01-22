#!/usr/bin/env python

import rospy, math, random
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

# Parameters
vel_max = 1.0
vel_min = 0.1
pos_lb = 0.5
pos_ub = 5.5
direction = 1.0

# Define global Twist message which is modified in the callback_base_pose_ground_truth and published in the main loop
twist_msg = Twist()

# This function manages turnarounds and new random velocities of the object
def callback_base_pose_ground_truth(base_pose_ground_truth):
  global twist_msg
  if base_pose_ground_truth.pose.pose.position.y >= pos_ub:
    twist_msg.linear.y = -direction * random.uniform(vel_min, vel_max) 
  if base_pose_ground_truth.pose.pose.position.y <= pos_lb:
    twist_msg.linear.y = direction * random.uniform(vel_min, vel_max)

# This function initializes the mover node and publishes continously a Twist message
def move_object():
  global pos_lb, pos_ub, vel_min, vel_max, direction
  
  rospy.init_node("obstacle_mover")

  # Lower bound of the obstacle movement in y-direction
  if rospy.has_param("~pos_lb"):
    pos_lb = rospy.get_param("~pos_lb")

  # Upper bound of the obstacle movement in y-direction
  if rospy.has_param("~pos_ub"):
    pos_ub = rospy.get_param("~pos_ub")

  # Velocity of movement in y direction
  if rospy.has_param("~vel_min"):
  	vel_min = rospy.get_param("~vel_min")

  if rospy.has_param("~vel_max"):
  	vel_max = rospy.get_param("~vel_max")

  if rospy.has_param("~direction"):
  	vel_max = rospy.get_param("~direction")
  	
  # initialize movement with random velocity in random direction
  twist_msg.linear.y = float(random.choice(['-1', '1'])) * random.uniform(vel_min, vel_max)

  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('base_pose_ground_truth', Odometry, callback_base_pose_ground_truth)
  
  r = rospy.Rate(10)
  # publish movement command continuously
  while not rospy.is_shutdown():
    pub.publish(twist_msg)
    r.sleep()

if __name__ == '__main__': 
  try:
    move_object()
  except rospy.ROSInterruptException:
    pass
