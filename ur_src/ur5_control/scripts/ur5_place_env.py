#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import ctypes
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
import tf.transformations as tft
import time
import os

# Python 2/3 兼容性
try:
    # Python 2
    input = raw_input
except NameError:
    # Python 3
    pass

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
        
        # 定义关节名称
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # 定义初始姿态
        self.home_position = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        
        # 方块参数
        self.block_name = "block"  # Gazebo中的模型名称
        self.block_height = 0.05   # 方块高度(米)，可以根据实际情况调整
        
        # 放置位置
        self.place_position = [0.5, 0.3, 0.0]  # 放置位置 (x, y, z)
        
        # 垂直朝下的旋转矩阵
        self.downward_rotation_matrix = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])
        
        # 存储当前关节角度
        self.current_joint_values = list(self.home_position)
        
        # 吸盘偏移
        self.suction_offset = 0.2  # 吸盘与末端执行器的偏移距离(米)
        
        # 移动到初始位置
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("UR5已移动到初始位置")
        rospy.loginfo("初始化完成")

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
        
        # 更新当前关节角度
        self.current_joint_values = list(positions)
    
    def forward_kinematics(self, joint_angles):
        """计算正运动学"""
        # 创建输入数组
        joints = (ctypes.c_double * 6)()
        for i in range(6):
            joints[i] = joint_angles[i]
        
        # 创建输出数组（4x4矩阵 = 16个元素）
        pose_matrix = (ctypes.c_double * 16)()
        
        # 调用函数
        self.forward_func(joints, pose_matrix)
        
        # 转换结果为numpy数组
        result = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                result[i, j] = pose_matrix[i*4 + j]
        
        return result
    
    def inverse_kinematics(self, trans_matrix, q_guess=None):
        """计算逆运动学"""
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
    
    def best_sol(self, sols, q_guess, weights=None):
        """从多个IK解中选择最佳解"""
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
    
    def get_block_position(self):
        """获取方块的位置信息"""
        try:
            model_state = self.get_model_state(self.block_name, "world")
            if not model_state.success:
                rospy.logerr("无法获取{}的位置".format(self.block_name))
                return None
            
            # 获取方块的位置
            x = model_state.pose.position.x
            y = model_state.pose.position.y
            z = model_state.pose.position.z
            
            # 计算方块顶部中心点
            top_center = [x, y, z + self.block_height/2.0]
            
            rospy.loginfo("方块位置: [{:.3f}, {:.3f}, {:.3f}]".format(x, y, z))
            rospy.loginfo("方块顶部中心点: [{:.3f}, {:.3f}, {:.3f}]".format(top_center[0], top_center[1], top_center[2]))
            
            return top_center
        except rospy.ServiceException as e:
            rospy.logerr("获取方块位置服务调用失败: {}".format(e))
            return None
    
    def create_downward_transform(self, position):
        """创建一个末端执行器垂直向下的变换矩阵"""
        transform = np.eye(4)
        transform[:3, :3] = self.downward_rotation_matrix
        transform[0, 3] = position[0]
        transform[1, 3] = position[1]
        transform[2, 3] = position[2]
        return transform
    
    def move_to_cartesian_pose(self, position, duration=2.0):
        """移动到指定的笛卡尔位置，保持末端执行器垂直朝下"""
        rospy.loginfo("移动到位置: [{:.3f}, {:.3f}, {:.3f}]".format(position[0], position[1], position[2]))
        
        # 创建垂直向下的变换矩阵
        transform = self.create_downward_transform(position)
        
        # 计算逆运动学
        joint_solution = self.inverse_kinematics(transform, self.current_joint_values)
        
        if joint_solution is None:
            rospy.logerr("无法找到IK解")
            return False
        
        # 移动到目标位置
        self.move_to_joint_positions(joint_solution, duration)
        return True
    
    def pick_block(self):
        """抓取方块"""
        rospy.loginfo("开始抓取方块")
        
        # 获取方块顶部中心点
        top_center = self.get_block_position()
        if top_center is None:
            rospy.logerr("无法获取方块位置")
            return False
        
        # 计算预抓取位置 (在方块顶部上方10cm)
        pre_pick_position = [top_center[0], top_center[1], top_center[2] + 0.2]
        
        # 移动到预抓取位置
        if not self.move_to_cartesian_pose(pre_pick_position, duration=2.0):
            rospy.logerr("无法移动到预抓取位置")
            return False
        
        rospy.loginfo("已移动到预抓取位置")
        
        # 计算抓取位置 (精确到方块顶部)
        pick_position = [top_center[0], top_center[1], top_center[2] + self.suction_offset]
        
        # 移动到抓取位置
        if not self.move_to_cartesian_pose(pick_position, duration=1.0):
            rospy.logerr("无法移动到抓取位置")
            return False
        
        rospy.loginfo("已移动到抓取位置")
        
        # 模拟吸盘抓取
        rospy.loginfo("吸盘抓取方块...")
        time.sleep(1)  # 模拟抓取时间
        
        # 抬起方块
        if not self.move_to_cartesian_pose(pre_pick_position, duration=1.0):
            rospy.logerr("抬起方块失败")
            return False
        
        rospy.loginfo("抓取完成，抬起方块")
        return True
    
    def place_block(self):
        """放置方块"""
        rospy.loginfo("开始放置方块")
        
        # 计算预放置位置 (在目标位置上方10cm)
        pre_place_position = [
            self.place_position[0], 
            self.place_position[1], 
            self.place_position[2] + self.block_height + 0.1
        ]
        
        # 移动到预放置位置
        if not self.move_to_cartesian_pose(pre_place_position, duration=2.0):
            rospy.logerr("无法移动到预放置位置")
            return False
        
        rospy.loginfo("已移动到预放置位置")
        
        # 计算放置位置
        place_position = [
            self.place_position[0], 
            self.place_position[1], 
            self.place_position[2] + self.block_height + self.suction_offset
        ]
        
        # 移动到放置位置
        if not self.move_to_cartesian_pose(place_position, duration=1.0):
            rospy.logerr("无法移动到放置位置")
            return False
        
        rospy.loginfo("已移动到放置位置")
        
        # 模拟释放 (更新方块位置)
        try:
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
        
        # 抬起机械臂
        if not self.move_to_cartesian_pose(pre_place_position, duration=1.0):
            rospy.logerr("抬起失败")
            return False
        
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
    
    def test_approach_block(self):
        """测试接近方块"""
        rospy.loginfo("测试接近方块")
        
        # 获取方块顶部中心点
        top_center = self.get_block_position()
        if top_center is None:
            rospy.logerr("无法获取方块位置")
            return False
        
        # 计算不同高度的测试位置
        heights = [0.2, 0.15, 0.1, 0.05, 0.02]
        
        for height in heights:
            test_position = [top_center[0], top_center[1], top_center[2] + height]
            rospy.loginfo("移动到高度 +{:.2f}m 的位置".format(height))
            
            if not self.move_to_cartesian_pose(test_position, duration=1.5):
                rospy.logerr("无法移动到测试位置")
                continue
                
            time.sleep(0.5)  # 暂停以便观察
        
        # 返回初始位置
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("测试完成")
        return True
    
    def run_interactive(self):
        """交互式菜单"""
        while not rospy.is_shutdown():
            print("\n===== UR5 抓取放置演示 =====")
            print("1. 移动到初始位置")
            print("2. 执行抓取和放置任务")
            print("3. 获取方块位置")
            print("4. 测试接近方块")
            print("5. 自定义放置位置")
            print("6. 退出")
            
            try:
                choice = int(input("请选择操作 (1-6): "))
                
                if choice == 1:
                    self.move_to_joint_positions(self.home_position)
                    rospy.loginfo("已移动到初始位置")
                
                elif choice == 2:
                    self.run_pick_and_place()
                
                elif choice == 3:
                    # 获取方块位置
                    top_center = self.get_block_position()
                    if top_center:
                        print("方块顶部中心点: {}".format(top_center))
                        
                        # 询问是否移动到顶部中心点上方
                        move_confirm = input("是否移动到方块顶部上方10cm? (y/n): ")
                        if move_confirm.lower() == 'y':
                            pre_grasp_pos = [top_center[0], top_center[1], top_center[2] + 0.1]
                            if self.move_to_cartesian_pose(pre_grasp_pos, duration=2.0):
                                print("已移动到方块顶部上方")
                            else:
                                print("无法移动到方块顶部上方")
                    else:
                        print("无法获取方块位置信息")
                
                elif choice == 4:
                    self.test_approach_block()
                
                elif choice == 5:
                    x = float(input("输入放置X坐标 (米): "))
                    y = float(input("输入放置Y坐标 (米): "))
                    z = float(input("输入放置Z坐标 (米): "))
                    self.place_position = [x, y, z]
                    rospy.loginfo("放置位置已更新: [{}, {}, {}]".format(x, y, z))
                
                elif choice == 6:
                    rospy.loginfo("程序结束")
                    break
                
                else:
                    rospy.logwarn("无效选择，请重新输入")
            
            except ValueError:
                rospy.logwarn("请输入有效的数字")
            except Exception as e:
                rospy.logerr("发生错误: {}".format(e))

if __name__ == '__main__':
    try:
        ur5_controller = UR5PickPlace()
        ur5_controller.run_interactive()
    except rospy.ROSInterruptException:
        pass

