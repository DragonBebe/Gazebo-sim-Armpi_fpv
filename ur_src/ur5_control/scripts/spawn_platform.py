#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_platform():
    rospy.init_node('spawn_platform')
    
    # 等待Gazebo服务
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    # 删除现有的平台（如果存在）
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("small_platform")
        rospy.loginfo("已删除现有平台")
    except rospy.ServiceException as e:
        rospy.logwarn("删除平台失败或平台不存在")
    
    # 创建平台的SDF
    platform_sdf = """
    <?xml version="1.0"?>
    <sdf version="1.5">
      <model name="small_platform">
        <pose>0 0 0 0 0 0</pose>
        <static>false</static>
        <link name="link">
          <inertial>
            <mass>0.5</mass>
            <inertia>
              <ixx>0.00108</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.00108</iyy>
              <iyz>0</iyz>
              <izz>0.0012</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <box>
                <size>0.12 0.12 0.02</size>
              </box>
            </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>0.8</mu>
                  <mu2>0.8</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1e5</kp>
                  <kd>1e2</kd>
                  <max_vel>0.1</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.12 0.12 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Green</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    
    # 设置平台的初始位置 - 在空中一定高度，让它自然下落
    initial_pose = Pose()
    initial_pose.position = Point(0.0, 0.5, 0.15)  # x, y, z 位置
    initial_pose.orientation = Quaternion(0, 0, 0, 1)  # 默认方向
    
    # 生成平台
    try:
        spawn_model("small_platform", platform_sdf, "", initial_pose, "world")
        rospy.loginfo("平台已成功生成，将自然下落到地面")
    except rospy.ServiceException as e:
        rospy.logerr("生成平台失败: %s" % e)
    
    rospy.loginfo("平台生成完成")

if __name__ == '__main__':
    try:
        spawn_platform()
    except rospy.ROSInterruptException:
        pass

