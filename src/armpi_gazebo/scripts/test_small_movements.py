#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import time

def test_small_movements():
    rospy.init_node('test_small_movements', anonymous=True)
    
    # 创建发布器
    pub_joint1 = rospy.Publisher('/armpi_fpv/joint1_position_controller/command', Float64, queue_size=1)
    
    # 等待发布器初始化
    rospy.sleep(1.0)
    
    # 测试joint1的小幅度移动
    rospy.loginfo("测试joint1的小幅度移动")
    
    # 先尝试移动到0位置
    rospy.loginfo("尝试移动到0位置")
    for _ in range(10):
        pub_joint1.publish(Float64(0.0))
        rospy.sleep(0.1)
    rospy.sleep(2.0)
    
    # 然后尝试小幅度正向移动
    rospy.loginfo("尝试小幅度正向移动")
    for i in range(10):
        position = 0.05 * (i + 1)  # 从0.05到0.5，每次增加0.05
        rospy.loginfo("发送位置命令: %f", position)
        pub_joint1.publish(Float64(position))
        rospy.sleep(1.0)  # 等待1秒观察效果
    
    # 然后尝试小幅度负向移动
    rospy.loginfo("尝试小幅度负向移动")
    for i in range(10):
        position = -0.05 * (i + 1)  # 从-0.05到-0.5，每次减少0.05
        rospy.loginfo("发送位置命令: %f", position)
        pub_joint1.publish(Float64(position))
        rospy.sleep(1.0)  # 等待1秒观察效果
    
    # 最后回到0位置
    rospy.loginfo("回到0位置")
    for _ in range(10):
        pub_joint1.publish(Float64(0.0))
        rospy.sleep(0.1)
    
    rospy.loginfo("测试完成")

if __name__ == '__main__':
    try:
        test_small_movements()
    except rospy.ROSInterruptException:
        pass

