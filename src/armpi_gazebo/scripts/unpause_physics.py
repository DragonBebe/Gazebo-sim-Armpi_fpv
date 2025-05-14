#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty
import time

def unpause_physics():
    rospy.init_node('unpause_physics')
    
    # 等待其他节点初始化
    time.sleep(5)
    
    # 调用服务取消暂停
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause()
        rospy.loginfo("物理引擎已取消暂停")
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s" % e)

if __name__ == '__main__':
    try:
        unpause_physics()
    except rospy.ROSInterruptException:
        pass

