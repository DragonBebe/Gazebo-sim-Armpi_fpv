#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python 2/3 兼容性
try:
    # Python 2
    input = raw_input
except NameError:
    # Python 3
    pass

import rospy
import actionlib
import numpy as np
import ctypes
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState, GetModelProperties
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft
import time
import os

class UR5PickPlace:
    def __init__(self):
        rospy.init_node('ur5_pick_place')
        
        # 加载UR5运动学库
        ur5_kin_lib_path = "/home/dragon/ur_ws/devel/lib/libur5_kin.so"
        if not os.path.exists(ur5_kin_lib_path):
            rospy.logerr("错误：找不到库文件 {}".format(ur5_kin_lib_path))
            return
        
        try:
            rospy.loginfo("正在加载库文件: {}".format(ur5_kin_lib_path))
            self.ur5_kin = ctypes.CDLL(ur5_kin_lib_path)
            rospy.loginfo("库文件加载成功！")
            
            # 获取函数
            self.forward_func = getattr(self.ur5_kin, "_ZN13ur_kinematics7forwardEPKdPd")
            self.inverse_func = getattr(self.ur5_kin, "_ZN13ur_kinematics7inverseEPKdPdd")
            
            # 设置函数签名
            self.forward_func.argtypes = [
                ctypes.POINTER(ctypes.c_double),  # 输入：关节角度数组
                ctypes.POINTER(ctypes.c_double)   # 输出：末端位姿矩阵
            ]
            self.forward_func.restype = ctypes.c_int
            
            self.inverse_func.argtypes = [
                ctypes.POINTER(ctypes.c_double),  # 输入：末端位姿矩阵
                ctypes.POINTER(ctypes.c_double),  # 输出：关节角度数组
                ctypes.c_double                   # 阈值参数
            ]
            self.inverse_func.restype = ctypes.c_int
            
            rospy.loginfo("UR5运动学函数已设置")
        except Exception as e:
            rospy.logerr("加载UR5运动学库失败: {}".format(e))
            return
        
        # 连接到UR5控制器的action server
        self.client = actionlib.SimpleActionClient(
            '/eff_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("等待action server...")
        self.client.wait_for_server()
        rospy.loginfo("已连接到action server")
        
        # 获取物体位置的服务
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # 设置物体位置的服务
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 获取模型属性的服务
        try:
            rospy.wait_for_service('/gazebo/get_model_properties', timeout=5.0)
            self.get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            rospy.loginfo("已连接到get_model_properties服务")
        except rospy.ROSException:
            rospy.logwarn("无法连接到get_model_properties服务，将使用默认方块尺寸")
            self.get_model_properties = None
        
        # 订阅模型状态以获取更多信息
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.model_states = None
        
        # 定义关节名称
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # 定义初始姿态和预抓取姿态
        self.home_position = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        
        # 方块参数 (初始默认值)
        self.block_name = "block"  # Gazebo中的模型名称
        self.block_size = [0.05, 0.05, 0.05]  # 方块尺寸 [长, 宽, 高] (米)
        
        # 抓取和放置位置
        self.pick_position = [0.5, 0.0, 0.0]  # 方块的位置 (x, y, z)
        self.place_position = [0.5, 0.3, 0.0]  # 放置位置 (x, y, z)
        
        # 末端执行器参数
        self.gripper_length = 0.02  # 夹爪长度(米)
        self.gripper_approach_distance = 0.1  # 预抓取高度(米)
        
        # 移动到初始位置
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("UR5已移动到初始位置")
        
        # 等待模型状态更新
        rospy.sleep(1.0)

    def model_states_callback(self, msg):
        """接收模型状态更新"""
        self.model_states = msg

    def get_block_dimensions(self):
        """
        获取方块的尺寸信息
        返回：[长, 宽, 高] (米)
        """
        # 尝试从碰撞箱或视觉模型中获取尺寸
        # 注意：这是一个近似方法，实际应用中可能需要更精确的方法
        
        try:
            # 首先尝试从模型状态中获取
            if self.model_states:
                try:
                    block_index = self.model_states.name.index(self.block_name)
                    # 这里我们无法直接从模型状态获取尺寸，但可以记录位置
                    rospy.loginfo("在模型状态中找到方块")
                except ValueError:
                    rospy.logwarn("在模型状态中未找到方块")
            
            # 获取方块当前位置
            model_state = self.get_model_state(self.block_name, "world")
            if not model_state.success:
                rospy.logwarn("无法获取{}的位置，使用默认尺寸".format(self.block_name))
                return self.block_size
                
            # 这里我们可以尝试使用URDF解析或其他方法获取尺寸
            # 但在此示例中，我们使用一个简单的启发式方法
            
            # 假设方块是立方体，尝试通过多次采样不同位置来估计尺寸
            # 实际应用中，应该从URDF/SDF文件或Gazebo API获取
            
            rospy.loginfo("使用默认方块尺寸: {}".format(self.block_size))
            return self.block_size
            
        except Exception as e:
            rospy.logerr("获取方块尺寸时出错: {}".format(e))
            return self.block_size  # 返回默认值

    def get_block_top_center(self):
        """
        获取方块顶部中心点的坐标
        返回：[x, y, z] 坐标 (米)
        """
        try:
            # 获取方块当前位置
            model_state = self.get_model_state(self.block_name, "world")
            if not model_state.success:
                rospy.logerr("无法获取{}的位置".format(self.block_name))
                return None
            
            block_pose = model_state.pose
            block_dimensions = self.get_block_dimensions()
            
            # 计算顶部中心点 (考虑方块的方向)
            # 注意：这里假设方块没有旋转，如果有旋转需要考虑四元数
            
            # 获取方块的位置
            block_position = [
                block_pose.position.x,
                block_pose.position.y,
                block_pose.position.z
            ]
            
            # 获取方块的方向(四元数)
            block_orientation = [
                block_pose.orientation.x,
                block_pose.orientation.y,
                block_pose.orientation.z,
                block_pose.orientation.w
            ]
            
            # 计算旋转矩阵
            rot_matrix = tft.quaternion_matrix(block_orientation)
            
            # 计算方块顶部中心点在方块坐标系中的位置 (假设原点在方块中心)
            local_top_center = [0, 0, block_dimensions[2]/2.0]
            
            # 将局部坐标转换为世界坐标
            world_top_center = np.dot(rot_matrix[:3,:3], local_top_center) + block_position
            
            rospy.loginfo("方块顶部中心点: {}".format(world_top_center))
            return world_top_center
            
        except Exception as e:
            rospy.logerr("计算方块顶部中心点时出错: {}".format(e))
            return None

    def best_sol(self, sols, q_guess, weights=None):
        """
        从多个IK解中选择最佳解
        
        参数:
        sols -- 逆运动学解的列表
        q_guess -- 当前关节角度
        weights -- 各关节的权重
        
        返回:
        最佳的关节角度解
        """
        if weights is None:
            weights = np.ones(6)
            
        valid_sols = []
        for sol in sols:
            test_sol = np.ones(6)*9999.
            for i in range(6):
                for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                    test_ang = sol[i] + add_ang
                    if (abs(test_ang) <= 2.*np.pi and 
                        abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                        test_sol[i] = test_ang
            if np.all(test_sol != 9999.):
                valid_sols.append(test_sol)
                
        if len(valid_sols) == 0:
            return None
            
        best_sol_ind = np.argmin(np.sum(weights*(np.array(valid_sols) - np.array(q_guess))**2, 1))
        return valid_sols[best_sol_ind]
    
    def move_to_joint_positions(self, positions, duration=2.0):
        """移动机械臂到指定的关节位置"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
    
    def move_through_waypoints(self, waypoints, durations=None):
        """
        通过一系列路径点移动机械臂
        
        参数:
        waypoints -- 关节角度列表的列表
        durations -- 每个路径点的持续时间列表(可选)
        """
        if not waypoints:
            rospy.logwarn("没有提供路径点")
            return
            
        if durations is None:
            durations = [2.0] * len(waypoints)
        elif len(durations) != len(waypoints):
            rospy.logwarn("路径点数量与持续时间数量不匹配，使用默认值")
            durations = [2.0] * len(waypoints)
            
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        time_from_start = 0.0
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            time_from_start += durations[i]
            point.time_from_start = rospy.Duration(time_from_start)
            goal.trajectory.points.append(point)
            
        self.client.send_goal(goal)
        self.client.wait_for_result()
    
    def forward_kinematics(self, joint_angles):
        """
        使用UR运动学库计算正运动学
        
        参数:
        joint_angles -- 关节角度列表
        
        返回:
        末端执行器的位置和姿态（4x4变换矩阵）
        """
        # 创建输入数组
        joints = (ctypes.c_double * 6)()
        for i in range(6):
            joints[i] = joint_angles[i]
        
        # 创建输出数组（4x4矩阵 = 16个元素）
        pose_matrix = (ctypes.c_double * 16)()
        
        # 调用函数
        status = self.forward_func(joints, pose_matrix)
        
        # 转换结果为numpy数组
        result = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                result[i, j] = pose_matrix[i*4 + j]
        
        return result
    
    def inverse_kinematics(self, trans_matrix, q_guess=None):
        """
        使用UR运动学库计算逆运动学
        
        参数:
        trans_matrix -- 4x4变换矩阵
        q_guess -- 当前关节角度猜测值
        
        返回:
        关节角度解列表
        """
        # 创建输入数组
        pose = (ctypes.c_double * 16)()
        for i in range(4):
            for j in range(4):
                pose[i*4 + j] = trans_matrix[i, j]
        
        # 创建输出数组（最多8组解，每组6个关节角度）
        solutions = (ctypes.c_double * 48)()  # 8*6 = 48
        
        # 调用函数
        threshold = 0.0
        num_sols = self.inverse_func(pose, solutions, ctypes.c_double(threshold))
        
        # 转换结果为Python列表
        sols = []
        for i in range(num_sols):
            sol = [solutions[i*6 + j] for j in range(6)]
            sols.append(sol)
        
        # 如果提供了猜测值，选择最佳解
        if q_guess is not None and len(sols) > 0:
            return self.best_sol(sols, q_guess)
        elif len(sols) > 0:
            return sols[0]  # 返回第一个解
        else:
            return None
    
    def calculate_ik_for_cartesian_pose(self, position, orientation=None):
        """
        计算给定笛卡尔位置和方向的逆运动学
        
        参数:
        position -- [x, y, z] 位置
        orientation -- [x, y, z, w] 四元数方向，默认为末端朝下
        
        返回:
        关节角度列表
        """
        if orientation is None:
            # 默认姿态：末端朝下
            orientation = [0, 1, 0, 0]  # 绕x轴旋转180度
            
        # 创建4x4变换矩阵
        trans_matrix = np.eye(4)
        
        # 设置旋转部分（从四元数）
        qx, qy, qz, qw = orientation
        rot_matrix = tft.quaternion_matrix([qw, qx, qy, qz])  # 注意四元数顺序
        trans_matrix[:3, :3] = rot_matrix[:3, :3]
        
        # 设置平移部分
        trans_matrix[0, 3] = position[0]
        trans_matrix[1, 3] = position[1]
        trans_matrix[2, 3] = position[2]
        
        # 计算逆运动学
        current_joints = np.array(self.home_position)  # 使用当前关节位置作为参考
        q_sol = self.inverse_kinematics(trans_matrix, current_joints)
        
        if q_sol is None:
            rospy.logerr("无法找到有效的IK解")
            return None
            
        return q_sol.tolist() if isinstance(q_sol, np.ndarray) else q_sol
    
    def plan_pick_trajectory(self, top_center):
        """
        规划抓取方块的轨迹
        
        参数:
        top_center -- 方块顶部中心点坐标 [x, y, z]
        
        返回:
        轨迹路径点列表和时间列表
        """
        # 计算预抓取位置 (在方块上方)
        pre_pick_position = [
            top_center[0], 
            top_center[1], 
            top_center[2] + self.gripper_approach_distance
        ]
        
        # 计算中间位置 (降低高度)
        mid_pick_position = [
            top_center[0], 
            top_center[1], 
            top_center[2] + self.gripper_approach_distance/2.0
        ]
        
        # 计算抓取位置 (略高于方块顶部)
        pick_position = [
            top_center[0], 
            top_center[1], 
            top_center[2] + self.gripper_length
        ]
        
        # 计算各个位置的IK解
        pre_pick_joints = self.calculate_ik_for_cartesian_pose(pre_pick_position)
        if pre_pick_joints is None:
            rospy.logerr("无法计算预抓取位置的IK解")
            return None, None
            
        mid_pick_joints = self.calculate_ik_for_cartesian_pose(mid_pick_position)
        if mid_pick_joints is None:
            rospy.logerr("无法计算中间抓取位置的IK解")
            return None, None
            
        pick_joints = self.calculate_ik_for_cartesian_pose(pick_position)
        if pick_joints is None:
            rospy.logerr("无法计算抓取位置的IK解")
            return None, None
            
        # 创建轨迹
        waypoints = [pre_pick_joints, mid_pick_joints, pick_joints]
        durations = [2.0, 1.5, 1.0]  # 调整时间以获得平滑运动
        
        return waypoints, durations
    
    def pick_block(self):
        """抓取方块的操作序列"""
        rospy.loginfo("开始抓取方块")
        
        # 获取方块顶部中心点
        top_center = self.get_block_top_center()
        if top_center is None:
            rospy.logerr("无法获取方块顶部中心点")
            return False
        
        # 规划抓取轨迹
        pick_waypoints, pick_durations = self.plan_pick_trajectory(top_center)
        if pick_waypoints is None:
            rospy.logerr("无法规划抓取轨迹")
            return False
        
        # 移动到预抓取位置
        self.move_to_joint_positions(pick_waypoints[0])
        rospy.loginfo("已移动到预抓取位置")
        
        # 执行抓取轨迹
        self.move_through_waypoints(pick_waypoints[1:], pick_durations[1:])
        rospy.loginfo("已移动到抓取位置")
        
        # 模拟抓取 (将方块附着到机械臂)
        rospy.loginfo("抓取方块...")
        time.sleep(1)  # 模拟抓取时间
        
        # 移动回预抓取位置 (带着方块)
        self.move_to_joint_positions(pick_waypoints[0])
        rospy.loginfo("抓取完成，抬起方块")
        
        return True
    
    def plan_place_trajectory(self, place_position):
        """
        规划放置方块的轨迹
        
        参数:
        place_position -- 放置位置坐标 [x, y, z]
        
        返回:
        轨迹路径点列表和时间列表
        """
        # 获取方块尺寸
        block_dimensions = self.get_block_dimensions()
        
        # 计算预放置位置 (在目标位置上方)
        pre_place_position = [
            place_position[0], 
            place_position[1], 
            place_position[2] + block_dimensions[2] + self.gripper_approach_distance
        ]
        
        # 计算中间位置 (降低高度)
        mid_place_position = [
            place_position[0], 
            place_position[1], 
            place_position[2] + block_dimensions[2] + self.gripper_approach_distance/2.0
        ]
        
        # 计算放置位置 (方块底部接触地面)
        place_position_adjusted = [
            place_position[0], 
            place_position[1], 
            place_position[2] + block_dimensions[2] + self.gripper_length
        ]
        
        # 计算各个位置的IK解
        pre_place_joints = self.calculate_ik_for_cartesian_pose(pre_place_position)
        if pre_place_joints is None:
            rospy.logerr("无法计算预放置位置的IK解")
            return None, None
            
        mid_place_joints = self.calculate_ik_for_cartesian_pose(mid_place_position)
        if mid_place_joints is None:
            rospy.logerr("无法计算中间放置位置的IK解")
            return None, None
            
        place_joints = self.calculate_ik_for_cartesian_pose(place_position_adjusted)
        if place_joints is None:
            rospy.logerr("无法计算放置位置的IK解")
            return None, None
            
        # 创建轨迹
        waypoints = [pre_place_joints, mid_place_joints, place_joints]
        durations = [2.0, 1.5, 1.0]  # 调整时间以获得平滑运动
        
        return waypoints, durations
    
    def place_block(self):
        """放置方块的操作序列"""
        rospy.loginfo("开始放置方块")
        
        # 规划放置轨迹
        place_waypoints, place_durations = self.plan_place_trajectory(self.place_position)
        if place_waypoints is None:
            rospy.logerr("无法规划放置轨迹")
            return False
        
        # 移动到预放置位置
        self.move_to_joint_positions(place_waypoints[0])
        rospy.loginfo("已移动到预放置位置")
        
        # 执行放置轨迹
        self.move_through_waypoints(place_waypoints[1:], place_durations[1:])
        rospy.loginfo("已移动到放置位置")
        
        # 模拟放置 (更新方块位置)
        try:
            # 获取方块尺寸
            block_dimensions = self.get_block_dimensions()
            
            model_state = ModelState()
            model_state.model_name = self.block_name
            model_state.pose.position.x = self.place_position[0]
            model_state.pose.position.y = self.place_position[1]
            model_state.pose.position.z = self.place_position[2]
            model_state.pose.orientation.w = 1.0
            model_state.reference_frame = "world"
            
            self.set_model_state(model_state)
            rospy.loginfo("方块已放置到新位置")
        except rospy.ServiceException as e:
            rospy.logerr("设置模型状态服务调用失败: {}".format(e))
            return False
        
        # 移动回预放置位置
        self.move_to_joint_positions(place_waypoints[0])
        rospy.loginfo("放置完成，机械臂抬起")
        
        return True
    
    def run_pick_and_place(self):
        """执行完整的抓取和放置任务"""
        rospy.loginfo("开始执行抓取和放置任务")
        
        # 移动到初始位置
        self.move_to_joint_positions(self.home_position)
        
        # 抓取方块
        if not self.pick_block():
            rospy.logerr("抓取方块失败")
            return
        
        # 放置方块
        if not self.place_block():
            rospy.logerr("放置方块失败")
            return
        
        # 返回初始位置
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("任务完成，已返回初始位置")
    
    def run_interactive(self):
        """交互式菜单"""
        while not rospy.is_shutdown():
            print("\n===== UR5 抓取放置演示 =====")
            print("1. 移动到初始位置")
            print("2. 执行抓取和放置任务")
            print("3. 自定义方块位置")
            print("4. 自定义放置位置")
            print("5. 测试运动学")
            print("6. 获取方块信息")
            print("7. 退出")
            
            try:
                choice = int(input("请选择操作 (1-7): "))
                
                if choice == 1:
                    self.move_to_joint_positions(self.home_position)
                    rospy.loginfo("已移动到初始位置")
                
                elif choice == 2:
                    self.run_pick_and_place()
                
                elif choice == 3:
                    x = float(input("输入方块X坐标 (米): "))
                    y = float(input("输入方块Y坐标 (米): "))
                    z = float(input("输入方块Z坐标 (米): "))
                    
                    # 更新Gazebo中方块的位置
                    try:
                        model_state = ModelState()
                        model_state.model_name = self.block_name
                        model_state.pose.position.x = x
                        model_state.pose.position.y = y
                        model_state.pose.position.z = z
                        model_state.pose.orientation.w = 1.0
                        model_state.reference_frame = "world"
                        
                        self.set_model_state(model_state)
                        rospy.loginfo("方块位置已更新: [{}, {}, {}]".format(x, y, z))
                    except rospy.ServiceException as e:
                        rospy.logerr("设置模型状态服务调用失败: {}".format(e))
                
                elif choice == 4:
                    x = float(input("输入放置X坐标 (米): "))
                    y = float(input("输入放置Y坐标 (米): "))
                    z = float(input("输入放置Z坐标 (米): "))
                    self.place_position = [x, y, z]
                    rospy.loginfo("放置位置已更新: [{}, {}, {}]".format(x, y, z))
                
                elif choice == 5:
                    # 测试运动学计算
                    print("\n--- 运动学测试 ---")
                    print("1. 测试正运动学")
                    print("2. 测试逆运动学")
                    test_choice = int(input("请选择: "))
                    
                    if test_choice == 1:
                        # 测试正运动学
                        joint_angles = []
                        for i in range(6):
                            angle = float(input("输入关节 {} 角度 (弧度): ".format(i+1)))
                            joint_angles.append(angle)
                        
                        trans_matrix = self.forward_kinematics(joint_angles)
                        position = trans_matrix[:3, 3]
                        rotation = tft.quaternion_from_matrix(trans_matrix)
                        
                        print("末端位置: {}".format(position))
                        print("末端姿态 (四元数): {}".format(rotation))
                    
                    elif test_choice == 2:
                        # 测试逆运动学
                        x = float(input("输入X坐标 (米): "))
                        y = float(input("输入Y坐标 (米): "))
                        z = float(input("输入Z坐标 (米): "))
                        
                        position = [x, y, z]
                        joint_angles = self.calculate_ik_for_cartesian_pose(position)
                        
                        if joint_angles:
                            print("IK解: {}".format(joint_angles))
                            move_confirm = input("是否移动到该位置? (y/n): ")
                            if move_confirm.lower() == 'y':
                                self.move_to_joint_positions(joint_angles)
                        else:
                            print("无法计算IK解")
                
                elif choice == 6:
                    # 获取方块信息
                    print("\n--- 方块信息 ---")
                    
                    # 获取方块尺寸
                    dimensions = self.get_block_dimensions()
                    print("方块尺寸 [长, 宽, 高]: {}".format(dimensions))
                    
                    # 获取方块位置
                    try:
                        model_state = self.get_model_state(self.block_name, "world")
                        if model_state.success:
                            block_pose = model_state.pose
                            print("方块位置: [{}, {}, {}]".format(
                                block_pose.position.x,
                                block_pose.position.y,
                                block_pose.position.z
                            ))
                            print("方块方向 (四元数): [{}, {}, {}, {}]".format(
                                block_pose.orientation.x,
                                block_pose.orientation.y,
                                block_pose.orientation.z,
                                block_pose.orientation.w
                            ))
                            
                            # 获取顶部中心点
                            top_center = self.get_block_top_center()
                            if top_center is not None:
                                print("方块顶部中心点: {}".format(top_center))
                                
                                # 询问是否移动到顶部中心点上方
                                move_confirm = input("是否移动到方块顶部上方? (y/n): ")
                                if move_confirm.lower() == 'y':
                                    pre_grasp_pos = [top_center[0], top_center[1], top_center[2] + 0.1]
                                    joint_angles = self.calculate_ik_for_cartesian_pose(pre_grasp_pos)
                                    if joint_angles:
                                        self.move_to_joint_positions(joint_angles)
                                        print("已移动到方块顶部上方")
                                    else:
                                        print("无法计算移动到方块顶部上方的IK解")
                        else:
                            print("无法获取方块位置信息")
                    except rospy.ServiceException as e:
                        print("获取方块信息服务调用失败: {}".format(e))
                
                elif choice == 7:
                    rospy.loginfo("程序结束")
                    break
                
                else:
                    rospy.logwarn("无效选择，请重新输入")
            
            except ValueError:
                rospy.logwarn("请输入有效的数字")
            except Exception as e:
                rospy.logerr("发生错误: {}".format(e))
                import traceback
                traceback.print_exc()

if __name__ == '__main__':
    try:
        ur5_controller = UR5PickPlace()
        ur5_controller.run_interactive()
    except rospy.ROSInterruptException:
        pass


