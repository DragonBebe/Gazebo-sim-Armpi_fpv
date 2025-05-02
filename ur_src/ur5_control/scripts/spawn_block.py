#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_block():
    rospy.init_node('spawn_block')
    
    # 等待Gazebo服务
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    # 删除现有的方块（如果存在）
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("block")
        rospy.loginfo("已删除现有方块")
    except rospy.ServiceException as e:
        rospy.logwarn("删除方块失败或方块不存在")
    
    # 创建方块的SDF
    block_sdf = """
    <?xml version="1.0"?>
    <sdf version="1.5">
      <model name="block">
        <pose>0 0 0 0 0 0</pose>
        <static>false</static>
        <link name="link">
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.000021</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.000021</iyy>
              <iyz>0</iyz>
              <izz>0.000021</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>
              </box>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>1.0</mu>
                  <mu2>1.0</mu2>
                </ode>
              </friction>
            </surface>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    
    # 设置方块的初始位置
    initial_pose = Pose()
    initial_pose.position = Point(0.5, 0.0, 0.025)  # x, y, z
    initial_pose.orientation = Quaternion(0, 0, 0, 1)  # 默认方向
    
    # 生成方块
    try:
        spawn_model("block", block_sdf, "", initial_pose, "world")
        rospy.loginfo("方块已成功生成")
    except rospy.ServiceException as e:
        rospy.logerr("生成方块失败: %s" % e)
    
    rospy.loginfo("方块生成完成")

if __name__ == '__main__':
    try:
        spawn_block()
    except rospy.ROSInterruptException:
        pass

