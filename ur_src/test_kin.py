#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ctypes
import numpy as np
import os
import sys

def main():
    """
    使用ctypes加载并测试libur5_kin.so库的主函数
    """
    print("=== UR5 运动学库测试程序 ===")
    
    # 1. 加载库文件
    ur5_kin_lib_path = "/home/ubuntu/catkin_ws/devel/lib/libur5_kin.so"
    if not os.path.exists(ur5_kin_lib_path):
        print("错误：找不到库文件 {}".format(ur5_kin_lib_path))
        return False
    
    try:
        print("正在加载库文件: {}".format(ur5_kin_lib_path))
        ur5_kin = ctypes.CDLL(ur5_kin_lib_path)
        print("库文件加载成功！")
    except Exception as e:
        print("加载库文件失败: {}".format(e))
        return False
    
    # 2. 检查库中可用的函数
    print("\n=== 检查库中的函数 ===")
    
    # 根据nm命令输出，我们知道库中有以下函数:
    # _ZN13ur_kinematics7forwardEPKdPd
    # _ZN13ur_kinematics7inverseEPKdPdd
    # _ZN13ur_kinematics11forward_allEPKdPds2_S2_S2_S2_S2_
    
    # 这些是C++函数的符号名，实际在Python中调用时，我们需要使用未修饰的名称
    # 根据C++命名约定，这些函数可能对应于:
    # ur_kinematics::forward(const double*, double*)
    # ur_kinematics::inverse(const double*, double*, double)
    # ur_kinematics::forward_all(const double*, double*, ...)
    
    # 尝试获取这些函数
    try:
        # 由于C++命名空间的原因，函数名可能有多种可能性
        possible_forward_names = [
            "_ZN13ur_kinematics7forwardEPKdPd",  # 原始修饰名
            "ur_kinematics_forward",              # 可能的未修饰名
            "_ZN13ur_kinematics7forwardEPKdPd",  # 保持原样
            "forward"                            # 最简单的名称
        ]
        
        possible_inverse_names = [
            "_ZN13ur_kinematics7inverseEPKdPdd", # 原始修饰名
            "ur_kinematics_inverse",             # 可能的未修饰名
            "_ZN13ur_kinematics7inverseEPKdPdd", # 保持原样
            "inverse"                           # 最简单的名称
        ]
        
        # 尝试获取forward函数
        forward_func = None
        for name in possible_forward_names:
            try:
                forward_func = getattr(ur5_kin, name)
                print("找到forward函数: {}".format(name))
                break
            except AttributeError:
                pass
        
        if not forward_func:
            print("警告: 未找到forward函数")
        
        # 尝试获取inverse函数
        inverse_func = None
        for name in possible_inverse_names:
            try:
                inverse_func = getattr(ur5_kin, name)
                print("找到inverse函数: {}".format(name))
                break
            except AttributeError:
                pass
        
        if not inverse_func:
            print("警告: 未找到inverse函数")
            
    except Exception as e:
        print("检查函数时出错: {}".format(e))
    
    # 3. 设置函数签名
    print("\n=== 设置函数签名 ===")
    
    # 设置forward函数签名
    if forward_func:
        # 根据nm输出，forward函数签名为: (const double*, double*)
        # 即输入关节角度数组，输出末端位姿矩阵
        forward_func.argtypes = [
            ctypes.POINTER(ctypes.c_double),  # 输入：关节角度数组
            ctypes.POINTER(ctypes.c_double)   # 输出：末端位姿矩阵
        ]
        forward_func.restype = ctypes.c_int
        print("已设置forward函数签名")
    
    # 设置inverse函数签名
    if inverse_func:
        # 根据nm输出，inverse函数签名为: (const double*, double*, double)
        # 即输入末端位姿矩阵，输出关节角度数组，以及一个额外的double参数
        inverse_func.argtypes = [
            ctypes.POINTER(ctypes.c_double),  # 输入：末端位姿矩阵
            ctypes.POINTER(ctypes.c_double),  # 输出：关节角度数组
            ctypes.c_double                   # 可能是阈值或其他参数
        ]
        inverse_func.restype = ctypes.c_int
        print("已设置inverse函数签名")
    
    # 4. 测试正向运动学
    if forward_func:
        print("\n=== 测试正向运动学 ===")
        
        # 创建测试关节角度（单位：弧度）
        test_joint_angles = [0.0, -np.pi/2, 0.0, 0.0, 0.0, 0.0]
        print("测试关节角度(弧度): {}".format(test_joint_angles))
        print("测试关节角度(角度): {}".format([np.degrees(angle) for angle in test_joint_angles]))
        
        # 转换为C类型
        joint_array = (ctypes.c_double * 6)()
        for i in range(6):
            joint_array[i] = test_joint_angles[i]
        
        # 创建输出数组（4x4矩阵 = 16个元素）
        pose_matrix = (ctypes.c_double * 16)()
        
        # 调用函数
        try:
            status = forward_func(joint_array, pose_matrix)
            print("函数返回状态: {}".format(status))
            
            # 转换结果为numpy数组并显示
            result = np.zeros((4, 4))
            for i in range(4):
                for j in range(4):
                    result[i, j] = pose_matrix[i*4 + j]
            
            print("计算得到的末端位姿矩阵:")
            print(result)
            
            # 提取位置和姿态
            position = result[:3, 3]
            print("位置 (x, y, z): {}".format(position))
            
            # 从旋转矩阵计算欧拉角（ZYX顺序）
            from math import atan2, asin
            r = result[:3, :3]
            
            # 这是一个简化的欧拉角计算，实际应用中可能需要更健壮的方法
            try:
                beta = asin(-r[2, 0])
                alpha = atan2(r[2, 1]/np.cos(beta), r[2, 2]/np.cos(beta))
                gamma = atan2(r[1, 0]/np.cos(beta), r[0, 0]/np.cos(beta))
                euler = [alpha, beta, gamma]  # Roll, Pitch, Yaw
                print("欧拉角 (Roll, Pitch, Yaw) 弧度: {}".format(euler))
                print("欧拉角 (Roll, Pitch, Yaw) 角度: {}".format([np.degrees(angle) for angle in euler]))
            except:
                print("无法计算欧拉角（可能是奇异点）")
            
        except Exception as e:
            print("调用forward函数时出错: {}".format(e))
    
    # 5. 测试逆向运动学
    if inverse_func and 'result' in locals():
        print("\n=== 测试逆向运动学 ===")
        
        # 使用正向运动学的结果作为输入
        print("使用上面计算的位姿矩阵作为输入")
        
        # 转换为C类型
        pose_array = (ctypes.c_double * 16)()
        for i in range(4):
            for j in range(4):
                pose_array[i*4 + j] = result[i, j]
        
        # 创建输出数组（最多8组解，每组6个关节角度）
        solutions = (ctypes.c_double * 48)()  # 8*6 = 48
        
        # 调用函数 - 注意inverse函数需要第三个参数
        try:
            # 第三个参数可能是阈值或其他参数，这里使用0.0作为默认值
            threshold = 0.0
            num_sols = inverse_func(pose_array, solutions, ctypes.c_double(threshold))
            print("找到 {} 组解".format(num_sols))
            
            # 显示每组解
            for i in range(num_sols):
                sol = [solutions[i*6 + j] for j in range(6)]
                print("解 {} (弧度): {}".format(i+1, sol))
                print("解 {} (角度): {}".format(i+1, [np.degrees(angle) for angle in sol]))
                
                # 验证解的正确性
                if forward_func:
                    # 将这组解转换为C类型
                    verify_joints = (ctypes.c_double * 6)()
                    for j in range(6):
                        verify_joints[j] = sol[j]
                    
                    # 计算正向运动学
                    verify_pose = (ctypes.c_double * 16)()
                    forward_func(verify_joints, verify_pose)
                    
                    # 计算位置误差
                    pos_error = np.sqrt(
                        (verify_pose[3] - pose_array[3])**2 +
                        (verify_pose[7] - pose_array[7])**2 +
                        (verify_pose[11] - pose_array[11])**2
                    )
                    print("  位置误差: {:.6f} 米".format(pos_error))
                    
                print("")
                
        except Exception as e:
            print("调用inverse函数时出错: {}".format(e))
            print("错误详情: {}".format(e))
            # 尝试不同的调用方式
            try:
                print("尝试替代调用方式...")
                # 有些库可能使用不同的方式处理第三个参数
                num_sols = inverse_func(pose_array, solutions, threshold)
                print("替代调用成功，找到 {} 组解".format(num_sols))
            except Exception as e2:
                print("替代调用也失败: {}".format(e2))
    
    print("\n=== 测试完成 ===")
    return True

if __name__ == "__main__":
    main()

