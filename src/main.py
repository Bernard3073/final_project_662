#!/usr/bin/env python

from __future__ import print_function
from rosgraph.names import namespace # Printing
import rospy # Python client library
import actionlib # ROS action library
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal # Controller messages
from std_msgs.msg import Float64 # 64-bit floating point numbers
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint # Robot trajectories
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

d1 = 0.145
d2 = 0
d3 = 0.27
d4 = 0.135
d5 = 0.34
a1 = 0.2
a2 = -0.05
a3 = -0.05

# Initialize your ROS node
rospy.init_node("move_robot")
# parent frame for the listener
parent_frame = '/robot_base_velocity_controller/odom'
# child frame for the listener
child_frame = 'base_footprint'
pub = rospy.Publisher("/robot_base_velocity_controller/cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()
rate = rospy.Rate(4)

def create_trans_matrix(a, alpha, d, theta):
    alpha = np.deg2rad(alpha)
    # # spong
    # mat =  np.array([[ np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
    #                [ np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
    #                [ 0, np.sin(alpha), np.cos(alpha), d],
    #                [0, 0, 0, 1]])

    # craig
    mat =  np.array([[ np.cos(theta), -np.sin(theta), 0, a],
                   [ np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(theta)*d],
                   [ np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                   [0, 0, 0, 1]])

    return np.around(mat, decimals=4)

def jacobian(eff_end, T_mat_0n):
    o_0 = np.array([0, 0, 0])
    col_0_z = np.array([0, 0, 1])
    j_0_v = np.cross(col_0_z, eff_end - o_0)
    j_0_w = col_0_z
    jacobian_matrix = [np.concatenate([j_0_v, j_0_w])]

    for i in range(len(T_mat_0n)):
        o_i = np.around(np.array([T_mat_0n[i][0][3], T_mat_0n[i][1][3], T_mat_0n[i][2][3]]), decimals=2)
        col_z = np.around(np.array([T_mat_0n[i][0][2], T_mat_0n[i][1][2], T_mat_0n[i][2][2]]), decimals=2)
        j_v = np.cross(col_z.T, (eff_end - o_i).T)
        j_w = col_z
        j_i = np.concatenate([j_v, j_w])
        jacobian_matrix = np.vstack((jacobian_matrix, j_i))

    return np.around(jacobian_matrix.T, decimals=4)

def move_robot_arm(joint_values):
  """
  Function to move the robot arm to desired joint angles.
  :param: joint_values A list of desired angles for the joints of a robot arm 
  """

  joint_values.tolist()

  # Create the SimpleActionClient, passing the type of the action to the constructor
  arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
 
  # Wait for the server to start up and start listening for goals.
  arm_client.wait_for_server()
     
  # Create a new goal to send to the Action Server
  arm_goal = FollowJointTrajectoryGoal()
 
  # Store the names of each joint of the robot arm
  arm_goal.trajectory.joint_names = ['arm_base_joint', 'shoulder_joint','bottom_wrist_joint' ,'elbow_joint', 'top_wrist_joint']
   
  # Create a trajectory point   
  point = JointTrajectoryPoint()
 
  # Store the desired joint values
  point.positions = joint_values
 
  # Set the time it should in seconds take to move the arm to the desired joint angles
  point.time_from_start = rospy.Duration(3)
 
  # Add the desired joint values to the goal
  arm_goal.trajectory.points.append(point)
 
  # Define timeout values
  exec_timeout = rospy.Duration(10)
  prmpt_timeout = rospy.Duration(5)
 
  # Send a goal to the ActionServer and wait for the server to finish performing the action
  arm_client.send_goal_and_wait(arm_goal, exec_timeout, prmpt_timeout)

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/vacuum_gripper/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except(rospy.ServiceException, e):
        print("Service call failed: %s" % e)

def gripper_off():
    # Wait till the srv is available
    rospy.wait_for_service('/vacuum_gripper/off')
    try:
        # Create a handle for the calling the srv
        turn_off = rospy.ServiceProxy('/vacuum_gripper/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_off()
        return resp
    except(rospy.ServiceException, e):
        print("Service call failed: %s" % e)

def go_straight(dist, vel):
    distance_to_drive, linear_velocity = dist, vel
    # update linear.x from the command line
    velocity_msg.linear.x = linear_velocity
    # get the current time (s)
    t_0 = rospy.Time.now().to_sec()
    # keep track of the distance
    distance_moved = 0.0
    rate = rospy.Rate(3)
    # while the amount of distance has not been reached
    while distance_moved <= distance_to_drive:
        pub.publish(velocity_msg)
        rate.sleep()
        # time in sec in the loop
        t_1 = rospy.Time.now().to_sec()
        distance_moved = (t_1 - t_0) * abs(linear_velocity)

    rospy.loginfo("Distance reached")
    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)


def rotate(angle, vel):
    relative_angle_degree, angular_velocity = angle, vel
    velocity_msg.angular.z = angular_velocity
    
    t0 = rospy.Time.now().to_sec()
    while True:
        pub.publish(velocity_msg)
        rate.sleep()
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0) * angular_velocity
        if abs(current_angle_degree) >= np.deg2rad(abs(relative_angle_degree)):
            rospy.loginfo("angle is reached")
            break
    
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

def main():
  q_0 = np.array([0, 0, 0, 0, 0], dtype='float64')
  t_0 = 0
  delta_t = 0.1
  goal_pos = np.array([1, 0, 0])
  while t_0 <= 5:
    theta_1 = q_0[0]
    theta_2 = q_0[1]
    theta_3 = q_0[2]
    theta_4 = q_0[3]
    theta_5 = q_0[4]

    # Transformation Matrix
    T_mat_01 = create_trans_matrix(a1, 0, d1, theta_1)
    T_mat_12 = create_trans_matrix(a2, -90, d2, theta_2 + np.pi/2) 
    T_mat_23 = create_trans_matrix(a3, 0, d3, theta_3) 
    T_mat_34 = create_trans_matrix(0, 90, d4, theta_4) 
    T_mat_45 = create_trans_matrix(0, -90, d5, theta_5) 

    T_mat_02 = np.matmul(T_mat_01, T_mat_12)
    T_mat_03 = np.matmul(T_mat_02, T_mat_23)
    T_mat_04 = np.matmul(T_mat_03, T_mat_34)
    T_mat_05 = np.matmul(T_mat_04, T_mat_45)

    T_mat_0n = np.array([T_mat_01, T_mat_02, T_mat_03, T_mat_04])
    # print(T_mat_07)
    eff_end = np.around(np.array([T_mat_05[0][3], T_mat_05[1][3], T_mat_05[2][3]]), decimals=2)
    print(eff_end)
    if( 0.48 <= eff_end[0] <= 0.52 and 0.07 <= eff_end[1] <= 0.21 and 0.02 <= eff_end[2] <= 0.11): 
      break
    delta_p = goal_pos - eff_end
    vel_lin = delta_p / np.linalg.norm(delta_p) 
    end_eff_vel = np.around([vel_lin[0], vel_lin[1], vel_lin[2], 0, 0, 0], decimals=3)
    # print("end_eff_vel: ", end_eff_vel)
    jac_mat = jacobian(eff_end, T_mat_0n)
    # psuedo inverse
    jac_mat_inv = np.linalg.pinv(jac_mat)
    # print(jac_mat_inv)
    q_dot = np.around(np.matmul(jac_mat_inv, end_eff_vel), decimals=4)
    # q_0 += np.around(q_dot*delta_t, decimals=4)
    q_0 += np.around(np.clip(q_dot, -1*0.2, 0.2), decimals=4)
    move_robot_arm(q_0)
    t_0 += delta_t

  # Start Grasping Object
  gripper_on()
  move_robot_arm(np.array([q_0[0], q_0[1]+np.deg2rad(20), q_0[2], np.deg2rad(180), q_0[4]+np.deg2rad(-37.5)]))
  collect_object_angle = np.array([q_0[0], q_0[1] + np.deg2rad(-2.5),  q_0[2], np.deg2rad(180), q_0[4]+np.deg2rad(-37.5)])
  move_robot_arm(collect_object_angle)
  # Move to target position to place object
  rotate(220, 0.5)
  go_straight(0.55, 0.1)
  collect_object_angle = np.array([q_0[0]+np.deg2rad(-30), q_0[1] + np.deg2rad(15),  q_0[2], np.deg2rad(180), q_0[4]+np.deg2rad(-37.5)])
  move_robot_arm(collect_object_angle)
  gripper_off()
  collect_object_angle = np.array([q_0[0]+np.deg2rad(-30), q_0[1],  q_0[2], np.deg2rad(180), q_0[4]+np.deg2rad(-37.5)])
  go_straight(0.25, -0.1)

if __name__ == '__main__':
  try:
    main() 
     
  except rospy.ROSInterruptException:
    print("Program interrupted before completion.", file=sys.stderr)