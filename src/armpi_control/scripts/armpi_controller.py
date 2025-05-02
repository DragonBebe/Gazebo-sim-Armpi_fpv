#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def move_robot():
    # 初始化ROS节点
    rospy.init_node('armpi_controller_example')
    
    # 创建动作客户端
    arm_client = actionlib.SimpleActionClient(
        'eff_joint_traj_controller/follow_joint_trajectory', 
        FollowJointTrajectoryAction
    )
    gripper_client = actionlib.SimpleActionClient(
        'gripper_controller/follow_joint_trajectory', 
        FollowJointTrajectoryAction
    )
    
    # 等待服务器启动
    rospy.loginfo("Waiting for arm controller...")
    arm_client.wait_for_server()
    rospy.loginfo("Waiting for gripper controller...")
    gripper_client.wait_for_server()
    rospy.loginfo("Controllers connected!")
    
    # 创建机械臂轨迹目标
    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory = JointTrajectory()
    arm_goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    
    # 创建轨迹点
    point = JointTrajectoryPoint()
    point.positions = [0.5, 0.4, 0.3, 0.2, 0.1]  # 目标位置
    point.velocities = [0, 0, 0, 0, 0]  # 目标速度
    point.accelerations = [0, 0, 0, 0, 0]  # 目标加速度
    point.time_from_start = rospy.Duration(2.0)  # 执行时间
    
    arm_goal.trajectory.points.append(point)
    
    # 创建夹爪轨迹目标
    gripper_goal = FollowJointTrajectoryGoal()
    gripper_goal.trajectory = JointTrajectory()
    gripper_goal.trajectory.joint_names = ['r_joint', 'l_joint']
    
    # 创建轨迹点 - 夹爪关闭
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0.02, 0.02]  # 目标位置 - 根据实际夹爪调整
    gripper_point.velocities = [0, 0]
    gripper_point.accelerations = [0, 0]
    gripper_point.time_from_start = rospy.Duration(1.0)
    
    gripper_goal.trajectory.points.append(gripper_point)
    
    # 发送目标并等待完成
    rospy.loginfo("Moving arm...")
    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result(rospy.Duration(5.0))
    
    rospy.loginfo("Closing gripper...")
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(2.0))
    
    rospy.loginfo("Motion completed!")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
        sys.exit(0)

