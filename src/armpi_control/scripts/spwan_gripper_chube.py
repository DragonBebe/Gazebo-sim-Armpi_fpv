#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.srv import GetLinkState, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import xml.etree.ElementTree as ET
from xml.dom import minidom
import math

def get_link_position(link_name):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp = get_link_state(link_name, "world")
        return resp.link_state.pose.position
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def create_cube_sdf(width, height, depth, color="1 0 0 1", mass=0.01):
    sdf = ET.Element("sdf", version="1.5")
    model = ET.SubElement(sdf, "model", name="cube_target")
    pose = ET.SubElement(model, "pose")
    pose.text = "0 0 0 0 0 0"  # Will be set during spawn
    static = ET.SubElement(model, "static")
    static.text = "false"  # 设置为false使物体可移动
    
    link = ET.SubElement(model, "link", name="link")
    
    # 添加惯性属性
    inertial = ET.SubElement(link, "inertial")
    
    # 设置质量
    mass_elem = ET.SubElement(inertial, "mass")
    mass_elem.text = str(mass)
    
    # 计算立方体的惯性矩
    # 立方体的惯性矩: I = (1/12) * m * (h^2 + d^2) 对于x轴
    ixx = (1.0/12.0) * mass * (height**2 + depth**2)
    iyy = (1.0/12.0) * mass * (width**2 + depth**2)
    izz = (1.0/12.0) * mass * (width**2 + height**2)
    
    inertia = ET.SubElement(inertial, "inertia")
    
    ixx_elem = ET.SubElement(inertia, "ixx")
    ixx_elem.text = str(ixx)
    
    iyy_elem = ET.SubElement(inertia, "iyy")
    iyy_elem.text = str(iyy)
    
    izz_elem = ET.SubElement(inertia, "izz")
    izz_elem.text = str(izz)
    
    # 非对角元素为0
    ixy_elem = ET.SubElement(inertia, "ixy")
    ixy_elem.text = "0"
    
    ixz_elem = ET.SubElement(inertia, "ixz")
    ixz_elem.text = "0"
    
    iyz_elem = ET.SubElement(inertia, "iyz")
    iyz_elem.text = "0"
    
    collision = ET.SubElement(link, "collision", name="collision")
    geometry_c = ET.SubElement(collision, "geometry")
    box_c = ET.SubElement(geometry_c, "box")
    size_c = ET.SubElement(box_c, "size")
    size_c.text = "{} {} {}".format(width, height, depth)
    
    # 添加接触属性以改善抓取行为
    surface = ET.SubElement(collision, "surface")
    
    # 添加摩擦属性
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    
    mu = ET.SubElement(ode, "mu")
    mu.text = "1.0"  # 静摩擦系数
    
    mu2 = ET.SubElement(ode, "mu2")
    mu2.text = "1.0"  # 动摩擦系数
    
    # 添加接触参数
    contact = ET.SubElement(surface, "contact")
    ode_contact = ET.SubElement(contact, "ode")
    
    kp = ET.SubElement(ode_contact, "kp")
    kp.text = "1000000.0"  # 接触刚度
    
    kd = ET.SubElement(ode_contact, "kd")
    kd.text = "100.0"  # 阻尼系数
    
    visual = ET.SubElement(link, "visual", name="visual")
    geometry_v = ET.SubElement(visual, "geometry")
    box_v = ET.SubElement(geometry_v, "box")
    size_v = ET.SubElement(box_v, "size")
    size_v.text = "{} {} {}".format(width, height, depth)
    
    material = ET.SubElement(visual, "material")
    ambient = ET.SubElement(material, "ambient")
    ambient.text = color
    diffuse = ET.SubElement(material, "diffuse")
    diffuse.text = color
    
    # Python 2 兼容的 XML 字符串生成
    rough_string = ET.tostring(sdf)
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ").encode('utf-8')


def check_collision(cube_center, cube_size, joint_positions, safety_margin=0.02):
    """
    检查立方体是否与机械臂关节碰撞
    cube_center: 立方体中心点坐标 (x, y, z)
    cube_size: 立方体的边长
    joint_positions: 机械臂关节位置列表 [(x, y, z), ...]
    safety_margin: 安全距离，单位米
    返回: True表示有碰撞，False表示无碰撞
    """
    # 立方体的半边长
    half_size = cube_size / 2.0
    
    # 立方体的边界
    cube_min_x = cube_center[0] - half_size
    cube_max_x = cube_center[0] + half_size
    cube_min_y = cube_center[1] - half_size
    cube_max_y = cube_center[1] + half_size
    cube_min_z = cube_center[2] - half_size
    cube_max_z = cube_center[2] + half_size
    
    # 检查每个关节是否与立方体碰撞
    for joint_pos in joint_positions:
        # 计算关节到立方体中心的距离
        dx = max(cube_min_x - joint_pos.x, 0, joint_pos.x - cube_max_x)
        dy = max(cube_min_y - joint_pos.y, 0, joint_pos.y - cube_max_y)
        dz = max(cube_min_z - joint_pos.z, 0, joint_pos.z - cube_max_z)
        
        # 计算欧几里得距离
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 如果距离小于安全距离，则认为有碰撞
        if distance < safety_margin:
            rospy.logwarn("Potential collision detected with joint at ({}, {}, {})".format(
                joint_pos.x, joint_pos.y, joint_pos.z))
            return True
    
    return False

def get_all_joint_positions():
    """获取机械臂所有关节的位置"""
    joint_links = [
        "base_link", 
        "joint1_Link", 
        "joint2_Link", 
        "joint3_Link", 
        "joint4_Link",
        "joint5_Link",
        "joint6_Link"
    ]
    
    joint_positions = []
    for link in joint_links:
        pos = get_link_position(link)
        if pos:
            joint_positions.append(pos)
        else:
            rospy.logwarn("Failed to get position for link: {}".format(link))
    
    return joint_positions

def spawn_cube(x, y, z, width, height, depth):
    # 创建SDF
    sdf_str = create_cube_sdf(width, height, depth)
    
    # 设置位姿
    initial_pose = Pose()
    initial_pose.position = Point(x, y, z)
    initial_pose.orientation = Quaternion(0, 0, 0, 1)
    
    # 调用服务
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model("cube_target", sdf_str, "", initial_pose, "world")
        rospy.loginfo("Cube spawned successfully")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def get_gripper_tip_positions():
    """获取夹爪末端的位置"""
    # 获取夹爪连接点的位置
    r_out_link = get_link_position("r_out_link")
    l_out_link = get_link_position("l_out_link")
    
    if not r_out_link or not l_out_link:
        rospy.logerr("Failed to get gripper link positions")
        return None, None
    
    # 根据STL模型，计算夹爪末端的实际位置
    # 从图片可以看出，夹爪末端相对于连接点有一定偏移
    
    # 计算夹爪朝向向量（从中心指向外侧）
    center_x = (r_out_link.x + l_out_link.x) / 2
    center_y = (r_out_link.y + l_out_link.y) / 2
    center_z = (r_out_link.z + l_out_link.z) / 2
    
    # 计算右夹爪末端位置（根据STL模型调整偏移量）
    r_tip_offset = 0.01  # 减小偏移量，避免立方体位置过高
    r_dir_x = r_out_link.x - center_x
    r_dir_y = r_out_link.y - center_y
    r_dir_z = r_out_link.z - center_z
    
    # 归一化方向向量
    r_mag = math.sqrt(r_dir_x**2 + r_dir_y**2 + r_dir_z**2)
    if r_mag > 0:
        r_dir_x /= r_mag
        r_dir_y /= r_mag
        r_dir_z /= r_mag
    
    r_tip_x = r_out_link.x + r_dir_x * r_tip_offset
    r_tip_y = r_out_link.y + r_dir_y * r_tip_offset
    r_tip_z = r_out_link.z + r_dir_z * r_tip_offset
    
    # 计算左夹爪末端位置
    l_tip_offset = 0.01  # 减小偏移量，避免立方体位置过高
    l_dir_x = l_out_link.x - center_x
    l_dir_y = l_out_link.y - center_y
    l_dir_z = l_out_link.z - center_z
    
    # 归一化方向向量
    l_mag = math.sqrt(l_dir_x**2 + l_dir_y**2 + l_dir_z**2)
    if l_mag > 0:
        l_dir_x /= l_mag
        l_dir_y /= l_mag
        l_dir_z /= l_mag
    
    l_tip_x = l_out_link.x + l_dir_x * l_tip_offset
    l_tip_y = l_out_link.y + l_dir_y * l_tip_offset
    l_tip_z = l_out_link.z + l_dir_z * l_tip_offset
    
    # 创建Point对象表示夹爪末端位置
    r_tip = Point(r_tip_x, r_tip_y, r_tip_z)
    l_tip = Point(l_tip_x, l_tip_y, l_tip_z)
    
    return r_tip, l_tip

if __name__ == "__main__":
    rospy.init_node('spawn_gripper_cube')
    
    # 获取夹爪末端位置
    r_tip, l_tip = get_gripper_tip_positions()
    if not r_tip or not l_tip:
        rospy.logerr("Failed to calculate gripper tip positions")
        exit(1)
    
    # 计算夹爪末端之间的中心点（这是物体应该放置的位置）
    x_center = (r_tip.x + l_tip.x) / 2 
    y_center = (r_tip.y + l_tip.y) / 2
    z_center = (r_tip.z + l_tip.z) / 2
    height_offset = 0.01  # 增加5厘米的高度
    z_center = z_center + height_offset
    y_center = y_center + 0.03
	
    
    # 计算夹爪末端之间的距离
    distance = math.sqrt((r_tip.x - l_tip.x)**2 +
                         (r_tip.y - l_tip.y)**2 +
                         (r_tip.z - l_tip.z)**2)
    
    # 设置cube大小为夹爪距离的10%
    cube_size_x = distance * 0.3
    cube_size_y = distance * 0.3
    cube_size_z = distance * 0.8
    
    rospy.loginfo("Gripper tips center: ({:.4f}, {:.4f}, {:.4f})".format(x_center, y_center, z_center))
    rospy.loginfo("Adjusted cube position: ({:.4f}, {:.4f}, {:.4f})".format(x_center, y_center, z_center))
    rospy.loginfo("Gripper tips distance: {:.4f}".format(distance))
    rospy.loginfo("Cube size (10% of distance): {:.4f}".format(cube_size_x))
    
    # 获取所有关节位置
    joint_positions = get_all_joint_positions()
    
    # 检查是否会与关节碰撞
    if check_collision((x_center, y_center, z_center), cube_size_x, joint_positions):
        rospy.logerr("Cannot spawn cube: collision with robot joints detected")
        rospy.loginfo("Cube spawn canceled to prevent collision")
    else:
        rospy.loginfo("No collision detected, spawning cube...")
        # 生成cube
        spawn_cube(x_center, y_center, z_center, cube_size_x, cube_size_y, cube_size_z)

