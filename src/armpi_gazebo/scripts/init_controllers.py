#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
import time

def init_controllers():
    rospy.init_node('init_controllers', anonymous=True)
    
    # 创建发布器
    pub_joint1 = rospy.Publisher('/armpi_fpv/joint1_position_controller/command', Float64, queue_size=1)
    pub_joint2 = rospy.Publisher('/armpi_fpv/joint2_position_controller/command', Float64, queue_size=1)
    pub_joint3 = rospy.Publisher('/armpi_fpv/joint3_position_controller/command', Float64, queue_size=1)
    pub_joint4 = rospy.Publisher('/armpi_fpv/joint4_position_controller/command', Float64, queue_size=1)
    pub_joint5 = rospy.Publisher('/armpi_fpv/joint5_position_controller/command', Float64, queue_size=1)
    pub_gripper = rospy.Publisher('/armpi_fpv/gripper_controller/command', Float64, queue_size=1)
    
    # 等待发布器初始化
    rospy.sleep(2.0)
    
    # 发送初始位置命令
    for _ in range(10):  # 多次发送以确保命令被接收
        pub_joint1.publish(Float64(0.0))
        pub_joint2.publish(Float64(0.0))
        pub_joint3.publish(Float64(0.0))
        pub_joint4.publish(Float64(0.0))
        pub_joint5.publish(Float64(0.0))
        pub_gripper.publish(Float64(0.0))
        rospy.sleep(0.1)
    
    rospy.loginfo("初始位置命令已发送")

if __name__ == '__main__':
    try:
        init_controllers()
    except rospy.ROSInterruptException:
        pass

