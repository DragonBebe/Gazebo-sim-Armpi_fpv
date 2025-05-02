#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('ur5_controller')
    
    # 连接到action server
    client = actionlib.SimpleActionClient(
        '/eff_joint_traj_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    rospy.loginfo("等待action server...")
    client.wait_for_server()
    rospy.loginfo("已连接到action server")
    
    # 创建一个轨迹目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    
    # 添加一个轨迹点 - 例如移动到预定义位置
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]  # 示例位置
    point.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point)
    
    # 发送轨迹并等待完成
    rospy.loginfo("发送轨迹...")
    client.send_goal(goal)
    client.wait_for_result()
    
    # 保持节点运行，等待命令
    rospy.loginfo("轨迹执行完成，等待下一个命令...")
    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

