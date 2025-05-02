#!/usr/bin/python
# coding=utf8
import sys
import math
import rospy
import moveit_commander
import ik_transform
from tf.transformations import quaternion_from_euler, euler_from_quaternion

ik = ik_transform.ArmIK()

rospy.init_node('moveit_test')
moveit_commander.roscpp_initialize(sys.argv)

# 实例化，参数是在用moveit助手时设置的分组
arm = moveit_commander.MoveGroupCommander('arm')
gripper = moveit_commander.MoveGroupCommander('gripper')

# 参考点
reference_frame = 'base_link'

end_effector_link = arm.get_end_effector_link()
arm.set_end_effector_link(end_effector_link)
arm.set_pose_reference_frame(reference_frame)

arm.allow_replanning(True)

# 超时时间，单位s, 即规划和运行所用时间超过设定值时就返回超时提示
arm.set_planning_time(20)

# 位置误差单位m
arm.set_goal_position_tolerance(0.005)
# 姿态误差单位弧度
arm.set_goal_orientation_tolerance(0.01)

# 设置夹持器的开合角度，单位度，范围
def setGripperJoint(gripper_joint):
    gripper_goal = gripper.get_current_joint_values()
    gripper_goal[5] = math.radians(gripper_joint)
    
    gripper.go(gripper_goal)
    gripper.stop()
    gripper.clear_pose_targets()

# 设置各个关节角度，单位度
def setArmJoint(joint1, joint2, joint3, joint4, joint5):
    joint_goal = arm.get_current_joint_values()
    joint_goal[0] = math.radians(joint1)
    joint_goal[1] = math.radians(joint2)
    joint_goal[2] = math.radians(joint3)
    joint_goal[3] = math.radians(joint4)
    joint_goal[4] = math.radians(joint5)    
    
    arm.go(joint_goal)
    arm.stop()
    arm.clear_pose_targets()

# 以指定姿态移动到指定点
# moveit默认使用的是概率采用来进行路径规划，所以每次的路径可能都不一样
# 而且一样的点，可能每次成功失败的结果也可能不一样，如果失败了可以多次尝试
def setPosition(x, y, z, roll, pitch, yaw):
    # 参数分别为位置x,y,z单位m，姿态rpy，单位度
    arm_pose = arm.get_current_pose().pose
    arm_pose.position.x = x
    arm_pose.position.y = y
    arm_pose.position.z = z
    
    # 加入自定义逆解来找roll，提高成功率
    result = ik.setPitchRanges((x, y, z), roll, -180, 0)
    if result:
        roll = result[2]
    else:
        print('无法找到合适的roll')

    yaw = -math.atan2(x, -y)
    qua = quaternion_from_euler(math.radians(roll), math.radians(pitch), yaw)
    arm_pose.orientation.x = qua[0]
    arm_pose.orientation.y = qua[1]
    arm_pose.orientation.z = qua[2]
    arm_pose.orientation.w = qua[3]
     
    arm.set_start_state_to_current_state()
    arm.set_pose_target(arm_pose, end_effector_link)
    traj = arm.plan()
    arm.execute(traj)

if __name__ == '__main__':
    setGripperJoint(-80)
    setArmJoint(0, 30, -78, -101, 0)
    
    setArmJoint(-90, 30, -78, -101, 0)
    # 方法1:逆解由moveit计算，只提供roll， 速度慢，成功率低
    setPosition(0.28, 0, 0.125, -90, 0, 0)
    setPosition(0.28, 0, 0.215, -90, 0, 0)
    rospy.sleep(1)
    setGripperJoint(-80)
    setArmJoint(0, 30, -78, -101, 0)

    # 方法2:逆解自己计算，moveit只负责路径规划, 速度快，成功率高
    setArmJoint(-90, 30, -78, -101, 0)
    result = ik.setPitchRanges((0.28, 0, 0.125), -90, -180, 0)
    if result:
        setArmJoint(result[0]['theta6'], result[0]['theta5'], result[0]['theta4'], result[0]['theta3'], 0)
    rospy.sleep(1) 
    result = ik.setPitchRanges((0.28, 0, 0.215), -90, -180, 0)
    if result:
        setArmJoint(result[0]['theta6'], result[0]['theta5'], result[0]['theta4'], result[0]['theta3'], 0) 
    rospy.sleep(1)

    setGripperJoint(-80)
    setArmJoint(0, 30, -78, -101, 0)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
