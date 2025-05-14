#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import time

def reset_arm():
    rospy.init_node('reset_arm', anonymous=True)
    
    # 创建发布器
    pub_joint1 = rospy.Publisher('/armpi_fpv/joint1_position_controller/command', Float64, queue_size=1)
    pub_joint2 = rospy.Publisher('/armpi_fpv/joint2_position_controller/command', Float64, queue_size=1)
    pub_joint3 = rospy.Publisher('/armpi_fpv/joint3_position_controller/command', Float64, queue_size=1)
    pub_joint4 = rospy.Publisher('/armpi_fpv/joint4_position_controller/command', Float64, queue_size=1)
    pub_joint5 = rospy.Publisher('/armpi_fpv/joint5_position_controller/command', Float64, queue_size=1)
    pub_gripper = rospy.Publisher('/armpi_fpv/gripper_controller/command', Float64, queue_size=1)
    
    # 等待发布器初始化
    rospy.sleep(1.0)
    
    # 发送重置命令，尝试更小的值
    for i in range(20):  # 逐步移动到目标位置
        # 线性插值从当前位置到目标位置
        factor = float(i) / 19.0
        pub_joint1.publish(Float64(0.0 * factor))
        pub_joint2.publish(Float64(0.0 * factor))
        pub_joint3.publish(Float64(0.0 * factor))
        pub_joint4.publish(Float64(0.0 * factor))
        pub_joint5.publish(Float64(0.0 * factor))
        pub_gripper.publish(Float64(0.0 * factor))
        rospy.sleep(0.2)  # 每步等待200ms
    
    rospy.loginfo("机械臂已重置到初始位置")

if __name__ == '__main__':
    try:
        reset_arm()
    except rospy.ROSInterruptException:
        pass

