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

# Python 2/3 Compatibility
try:
    # Python 2
    input = raw_input
except NameError:
    # Python 3
    pass

class UR5PickPlace:
    def __init__(self):
        rospy.init_node('ur5_pick_place')
        
        # Load UR5 kinematics library
        ur5_kin_lib_path = "/home/dragon/ur_ws/devel/lib/libur5_kin.so"
        if not os.path.exists(ur5_kin_lib_path):
            rospy.logerr("Error: Library file not found {}".format(ur5_kin_lib_path))
            return
        
        try:
            rospy.loginfo("Loading library file: {}".format(ur5_kin_lib_path))
            self.ur5_kin = ctypes.CDLL(ur5_kin_lib_path)
            rospy.loginfo("Library file loaded successfully!")
            
            # Get functions
            self.forward_func = getattr(self.ur5_kin, "_ZN13ur_kinematics7forwardEPKdPd")
            self.inverse_func = getattr(self.ur5_kin, "_ZN13ur_kinematics7inverseEPKdPdd")
            
            # Set function signatures
            self.forward_func.argtypes = [
                ctypes.POINTER(ctypes.c_double),  # Input: Joint angle array
                ctypes.POINTER(ctypes.c_double)   # Output: End-effector pose matrix
            ]
            self.forward_func.restype = ctypes.c_int
            
            self.inverse_func.argtypes = [
                ctypes.POINTER(ctypes.c_double),  # Input: End-effector pose matrix
                ctypes.POINTER(ctypes.c_double),  # Output: Joint angle array
                ctypes.c_double                   # Threshold parameter
            ]
            self.inverse_func.restype = ctypes.c_int
            
            rospy.loginfo("UR5 kinematics functions set")
        except Exception as e:
            rospy.logerr("Failed to load UR5 kinematics library: {}".format(e))
            return
        
        # Connect to UR5 controller's action server
        self.client = actionlib.SimpleActionClient(
            '/eff_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to action server")
        
        # Service to get object position
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # Service to set object position
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Define joint names
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Define home position
        self.home_position = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        
        # Block parameters
        self.block_name = "block"  # Model name in Gazebo
        self.block_height = 0.05   # Block height (meters), adjustable according to actual conditions
        
        # Placement position
        self.place_position = [0.5, 0.3, 0.0]  # Placement position (x, y, z)
        
        # Downward rotation matrix
        self.downward_rotation_matrix = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])
        
        # Store current joint angles
        self.current_joint_values = list(self.home_position)
        
        # Suction cup offset
        self.suction_offset = 0.05  # Vertical offset distance between suction cup and end-effector (meters)
        self.suction_offset_ver = 0.02  # Horizontal offset distance between suction cup and end-effector (meters)
        
        # Move to initial position
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("UR5 moved to initial position")
        rospy.loginfo("Initialization complete")

    def move_to_joint_positions(self, positions, duration=2.0):
        """Move the robot arm to specified joint positions"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        # Update current joint angles
        self.current_joint_values = list(positions)
    
    def forward_kinematics(self, joint_angles):
        """Compute forward kinematics"""
        # Create input array
        joints = (ctypes.c_double * 6)()
        for i in range(6):
            joints[i] = joint_angles[i]
        
        # Create output array (4x4 matrix = 16 elements)
        pose_matrix = (ctypes.c_double * 16)()
        
        # Call function
        self.forward_func(joints, pose_matrix)
        
        # Convert result to numpy array
        result = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                result[i, j] = pose_matrix[i*4 + j]
        
        return result
    
    def inverse_kinematics(self, trans_matrix, q_guess=None):
        """Compute inverse kinematics"""
        # Create input array
        pose = (ctypes.c_double * 16)()
        for i in range(4):
            for j in range(4):
                pose[i*4 + j] = trans_matrix[i, j]
        
        # Create output array (up to 8 solutions, each with 6 joint angles)
        solutions = (ctypes.c_double * 48)()  # 8*6 = 48
        
        # Call function
        threshold = 0.0
        num_sols = self.inverse_func(pose, solutions, ctypes.c_double(threshold))
        
        # Convert result to Python list
        sols = []
        for i in range(num_sols):
            sol = [solutions[i*6 + j] for j in range(6)]
            sols.append(sol)
        
        # If a guess value is provided, select the best solution
        if q_guess is not None and len(sols) > 0:
            return self.best_sol(sols, q_guess)
        elif len(sols) > 0:
            return sols[0]  # Return the first solution
        else:
            return None
    
    def best_sol(self, sols, q_guess, weights=None):
        """Select the best solution from multiple IK solutions"""
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
        """Get the position of the block"""
        try:
            model_state = self.get_model_state(self.block_name, "world")
            if not model_state.success:
                rospy.logerr("Unable to get the position of {}".format(self.block_name))
                return None
            
            # Get the block's position
            x = model_state.pose.position.x
            y = model_state.pose.position.y
            z = model_state.pose.position.z
            
            # Calculate the top center point of the block
            top_center = [x, y, z + self.block_height/2.0]
            
            rospy.loginfo("Block position: [{:.3f}, {:.3f}, {:.3f}]".format(x, y, z))
            rospy.loginfo("Block top center point: [{:.3f}, {:.3f}, {:.3f}]".format(top_center[0], top_center[1], top_center[2]))
            
            return top_center
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service to get block position: {}".format(e))
            return None
    
    def create_downward_transform(self, position):
        """Create a transformation matrix for the end-effector to point downward"""
        transform = np.eye(4)
        transform[:3, :3] = self.downward_rotation_matrix
        transform[0, 3] = position[0]
        transform[1, 3] = position[1]
        transform[2, 3] = position[2]
        return transform
    
    def move_to_cartesian_pose(self, position, duration=2.0):
        """Move to the specified Cartesian position while keeping the end-effector pointing downward"""
        rospy.loginfo("Moving to position: [{:.3f}, {:.3f}, {:.3f}]".format(position[0], position[1], position[2]))
        
        # Create downward transformation matrix
        transform = self.create_downward_transform(position)
        
        # Compute inverse kinematics
        joint_solution = self.inverse_kinematics(transform, self.current_joint_values)
        
        if joint_solution is None:
            rospy.logerr("Unable to find IK solution")
            return False
        
        # Move to target position
        self.move_to_joint_positions(joint_solution, duration)
        return True
    
    def pick_block(self):
        """Pick up the block"""
        rospy.loginfo("Starting to pick up the block")
        
        # Get the top center point of the block
        top_center = self.get_block_position()
        if top_center is None:
            rospy.logerr("Unable to get block position")
            return False
        
        # Compute pre-pick position (10cm above the block's top center)
        pre_pick_position = [top_center[0], top_center[1], top_center[2] + 0.1]
        
        # Move to pre-pick position
        if not self.move_to_cartesian_pose(pre_pick_position, duration=2.0):
            rospy.logerr("Unable to move to pre-pick position")
            return False
        
        rospy.loginfo("Moved to pre-pick position")
        
        # Compute pick position (precisely at the block's top center)
        pick_position = [top_center[0] + self.suction_offset_ver, top_center[1] + self.suction_offset_ver, top_center[2] + self.suction_offset]
        
        # Move to pick position
        if not self.move_to_cartesian_pose(pick_position, duration=1.0):
            rospy.logerr("Unable to move to pick position")
            return False
        
        rospy.loginfo("Moved to pick position")
        
        # Simulate suction cup picking
        rospy.loginfo("Suction cup picking the block...")
        time.sleep(1)  # Simulate picking time
        
        # Lift the block
        if not self.move_to_cartesian_pose(pre_pick_position, duration=1.0):
            rospy.logerr("Failed to lift the block")
            return False
        
        rospy.loginfo("Picking complete, block lifted")
        return True
    
    def place_block(self):
        """Place the block"""
        rospy.loginfo("Starting to place the block")
        
        # Compute pre-place position (10cm above the target position)
        pre_place_position = [
            self.place_position[0], 
            self.place_position[1], 
            self.place_position[2] + self.block_height + 0.1
        ]
        
        # Move to pre-place position
        if not self.move_to_cartesian_pose(pre_place_position, duration=2.0):
            rospy.logerr("Unable to move to pre-place position")
            return False
        
        rospy.loginfo("Moved to pre-place position")
        
        # Compute place position
        place_position = [
            self.place_position[0], 
            self.place_position[1], 
            self.place_position[2] + self.block_height + self.suction_offset
        ]
        
        # Move to place position
        if not self.move_to_cartesian_pose(place_position, duration=1.0):
            rospy.logerr("Unable to move to place position")
            return False
        
        rospy.loginfo("Moved to place position")
        
        # Simulate release (update block position)
        try:
            model_state = ModelState()
            model_state.model_name = self.block_name
            model_state.pose.position.x = self.place_position[0]
            model_state.pose.position.y = self.place_position[1]
            model_state.pose.position.z = self.place_position[2]
            model_state.pose.orientation.w = 1.0
            model_state.reference_frame = "world"
            
            self.set_model_state(model_state)
            rospy.loginfo("Block placed at new position")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service to set model state: {}".format(e))
            return False
        
        # Lift the robot arm
        if not self.move_to_cartesian_pose(pre_place_position, duration=1.0):
            rospy.logerr("Failed to lift the arm")
            return False
        
        rospy.loginfo("Placement complete, arm lifted")
        return True
    
    def run_pick_and_place(self):
        """Execute the complete pick-and-place task"""
        rospy.loginfo("Starting pick-and-place task")
        
        # Move to initial position
        self.move_to_joint_positions(self.home_position)
        
        # Pick up the block
        if not self.pick_block():
            rospy.logerr("Failed to pick up the block")
            return
        
        # Place the block
        if not self.place_block():
            rospy.logerr("Failed to place the block")
            return
        
        # Return to initial position
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("Task complete, returned to initial position")
    
    def test_approach_block(self):
        """Test approaching the block"""
        rospy.loginfo("Testing block approach")
        
        # Get the top center point of the block
        top_center = self.get_block_position()
        if top_center is None:
            rospy.logerr("Unable to get block position")
            return False
        
        # Compute test positions at different heights
        heights = [0.2, 0.15, 0.1, 0.05, 0.02]
        
        for height in heights:
            test_position = [top_center[0], top_center[1], top_center[2] + height]
            rospy.loginfo("Moving to position at height +{:.2f}m".format(height))
            
            if not self.move_to_cartesian_pose(test_position, duration=1.5):
                rospy.logerr("Unable to move to test position")
                continue
                
            time.sleep(0.5)  # Pause for observation
        
        # Return to initial position
        self.move_to_joint_positions(self.home_position)
        rospy.loginfo("Test complete")
        return True
    
    def run_interactive(self):
        """Interactive menu"""
        while not rospy.is_shutdown():
            print("\n===== UR5 Pick-and-Place Demo =====")
            print("1. Move to initial position")
            print("2. Execute pick-and-place task")
            print("3. Get block position")
            print("4. Test block approach")
            print("5. Customize placement position")
            print("6. Exit")
            
            try:
                choice = int(input("Select an option (1-6): "))
                
                if choice == 1:
                    self.move_to_joint_positions(self.home_position)
                    rospy.loginfo("Moved to initial position")
                
                elif choice == 2:
                    self.run_pick_and_place()
                
                elif choice == 3:
                    # Get block position
                    top_center = self.get_block_position()
                    if top_center:
                        print("Block top center point: {}".format(top_center))
                        
                        # Ask whether to move above the top center point
                        move_confirm = input("Move to 10cm above the block's top center? (y/n): ")
                        if move_confirm.lower() == 'y':
                            pre_grasp_pos = [top_center[0], top_center[1], top_center[2] + 0.1]
                            if self.move_to_cartesian_pose(pre_grasp_pos, duration=2.0):
                                print("Moved to 10cm above the block's top center")
                            else:
                                print("Unable to move above the block's top center")
                    else:
                        print("Unable to get block position information")
                
                elif choice == 4:
                    self.test_approach_block()
                
                elif choice == 5:
                    x = float(input("Enter placement X coordinate (meters): "))
                    y = float(input("Enter placement Y coordinate (meters): "))
                    z = float(input("Enter placement Z coordinate (meters): "))
                    self.place_position = [x, y, z]
                    rospy.loginfo("Placement position updated: [{}, {}, {}]".format(x, y, z))
                
                elif choice == 6:
                    rospy.loginfo("Program terminated")
                    break
                
                else:
                    rospy.logwarn("Invalid selection, please try again")
            
            except ValueError:
                rospy.logwarn("Please enter a valid number")
            except Exception as e:
                rospy.logerr("Error occurred: {}".format(e))

if __name__ == '__main__':
    try:
        ur5_controller = UR5PickPlace()
        ur5_controller.run_interactive()
    except rospy.ROSInterruptException:
        pass

