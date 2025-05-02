#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class ArmPiController:
    def __init__(self):
        """初始化控制器"""
        rospy.init_node('armpi_position_controller')
        
        # 创建动作客户端
        self.arm_client = actionlib.SimpleActionClient(
            'eff_joint_traj_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        self.gripper_client = actionlib.SimpleActionClient(
            'gripper_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        # 等待服务器启动
        rospy.loginfo("Waiting for arm controller...")
        self.arm_client.wait_for_server()
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Controllers connected!")
        
        # 关节名称
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.gripper_joint_names = ['r_joint', 'l_joint']
    
    def move_to_position(self, positions, duration=2.0):
        """移动机械臂到指定位置
        
        Args:
            positions: 关节位置列表 [joint1, joint2, joint3, joint4, joint5]
            duration: 执行时间（秒）
        """
        if len(positions) != len(self.arm_joint_names):
            rospy.logerr("Position list length does not match joint count")
            return False
        
        # 创建轨迹目标
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.arm_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0] * len(positions)
        point.accelerations = [0] * len(positions)
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        # 发送目标
        rospy.loginfo("Moving arm to position: {}".format(positions))
        self.arm_client.send_goal(goal)
        
        # 等待完成
        return self.arm_client.wait_for_result(rospy.Duration(duration + 1.0))
    
    def control_gripper(self, position, duration=1.0):
        """控制夹爪
        
        Args:
            position: 夹爪位置（0.0为完全闭合，值越大越开）
            duration: 执行时间（秒）
        """
        # 创建轨迹目标
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.gripper_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = [position, position]  # 左右夹爪使用相同位置
        point.velocities = [0, 0]
        point.accelerations = [0, 0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        # 发送目标
        rospy.loginfo("Setting gripper position to: {}".format(position))
        self.gripper_client.send_goal(goal)
        
        # 等待完成
        return self.gripper_client.wait_for_result(rospy.Duration(duration + 1.0))
    
    def home_position(self):
        """将机械臂移动到初始位置"""
        return self.move_to_position([0, 0, 0, 0, 0])
    
    def open_gripper(self):
        """打开夹爪"""
        return self.control_gripper(0.1)  # 根据实际夹爪调整此值
    
    def close_gripper(self):
        """关闭夹爪"""
        return self.control_gripper(0.01)  # 根据实际夹爪调整此值

def main():
    """主函数"""
    controller = ArmPiController()
    
    try:
        # 示例动作序列
        rospy.loginfo("Moving to home position...")
        controller.home_position()
        rospy.sleep(1)
        
        rospy.loginfo("Opening gripper...")
        controller.open_gripper()
        rospy.sleep(1)
        
        rospy.loginfo("Moving to position 1...")
        controller.move_to_position([0.5, 0.4, 0.3, 0.2, 0.1])
        rospy.sleep(1)
        
        rospy.loginfo("Closing gripper...")
        controller.close_gripper()
        rospy.sleep(1)
        
        rospy.loginfo("Moving to position 2...")
        controller.move_to_position([0.2, 0.3, 0.4, 0.5, 0.6])
        rospy.sleep(1)
        
        rospy.loginfo("Opening gripper...")
        controller.open_gripper()
        rospy.sleep(1)
        
        rospy.loginfo("Returning to home position...")
        controller.home_position()
        
        rospy.loginfo("Motion sequence completed!")
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion")
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

