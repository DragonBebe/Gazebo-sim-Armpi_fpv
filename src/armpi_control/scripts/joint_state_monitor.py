#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    """回调函数，处理接收到的关节状态消息"""
    rospy.loginfo("Joint Names: {}".format(msg.name))
    rospy.loginfo("Joint Positions: {}".format(msg.position))
    rospy.loginfo("Joint Velocities: {}".format(msg.velocity))
    rospy.loginfo("Joint Efforts: {}".format(msg.effort))
    rospy.loginfo("-" * 50)

def monitor_joint_states():
    """监控关节状态"""
    rospy.init_node('joint_state_monitor')
    
    # 订阅关节状态话题
    rospy.Subscriber('joint_states', JointState, joint_states_callback)
    
    rospy.loginfo("Joint state monitor is running. Press Ctrl+C to stop.")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        monitor_joint_states()
    except rospy.ROSInterruptException:
        pass

