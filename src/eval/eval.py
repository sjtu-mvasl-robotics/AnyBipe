import os
import sys
file_path = os.path.dirname(os.path.abspath(__file__))
current_dir_path = os.path.dirname(file_path)
base_dir_path = os.path.dirname(current_dir_path)
library_path = os.path.join(base_dir_path, 'biped_ws', 'src', 'pointfoot-sdk-lowlevel', 'python3', 'amd64')
sys.path.append(library_path)
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread

# scipy for speed filtering
from scipy.signal import savgol_filter, butter, lfilter

# ros msgs
from geometry_msgs.msg import Twist

# gazebo
from gazebo_msgs.msg import ModelStates, LinkStates
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty


import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from functools import partial
from jacobian import py_state_vel_track, py_feet_distance_track
import subprocess
import signal

from scipy.spatial.transform import Rotation as R

import argparse


class SimEnvRobotTracker:
    def __init__(self, 
                 ip_addr = "127.0.0.1", 
                 node_name = "sim_robot_tracker",
                 robot_type = RobotType.PointFoot,
                 gazebo_imu_name = "point_foot_robot::base_Link",
                 export_path = None, 
                 project_path = "/home/yyf/limx_ws",
                 export_plot = True,
                 tracking_duration = 8): # 10 seconds
        
        # Initialize Robot object
        self.robot = None
        self.project_path = project_path
        self.robot_type = robot_type
        self.ip_addr = ip_addr

        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.export_path = export_path if export_path is not None else "exported_data/"
        if not os.path.exists(self.export_path):
            os.makedirs(self.export_path)

        self.gazebo_imu_name = gazebo_imu_name

        # IMU basics
        self.world_reset = False
        self.save_reset = False
        self.world_reset_count = 0
        self.sim_time = rospy.Time()
        self.t_current = time.time()
        self.t_prev = self.t_current

        self.gt_imu_pos = [[] for _ in range(3)]
        self.gt_imu_vel = [[] for _ in range(3)]
        self.gt_imu_acc = [[] for _ in range(3)]
        self.gt_imu_ang = [[] for _ in range(3)] # euler angles
        self.gt_imu_ang_vel = [[] for _ in range(3)]
        self.gt_t = []

        self.imu_t_current = 0
        self.imu_t_prev = 0
        #self.imu_pos = [[] for _ in range(3)]
        self.imu_vel = [[] for _ in range(3)]
        self.imu_acc = [[] for _ in range(3)]
        self.imu_ang = [[] for _ in range(3)]
        self.imu_ang_vel = [[] for _ in range(3)]
        self.imu_t = []

        # robot state
        self.q_init = [
                0,
                np.pi / 6,
                -np.pi / 3,
                0,
                np.pi / 6,
                -np.pi / 3
            ]
        self.l = [0.2368, 0.1863 + 0.0635]
        self.feet_original_offset = [0.0, 0.154, 0.0]
        self.rs_t_current = 0
        self.rs_t_prev = 0
        self.rs_t = []
        self.left_vel = [[] for _ in range(3)]
        self.right_vel = [[] for _ in range(3)]
        self.left_q = [[] for _ in range(3)]
        self.right_q = [[] for _ in range(3)]
        self.left_dq = [[] for _ in range(3)]
        self.right_dq = [[] for _ in range(3)]
        self.left_tau = [[] for _ in range(3)]
        self.right_tau = [[] for _ in range(3)]
        self.feet_state_left = [[] for _ in range(3)]
        self.feet_state_right = [[] for _ in range(3)]

        self.estimated_vel = [[] for _ in range(3)]


        self.ref_vel_x = np.random.uniform(0.2, 0.5)
        self.ref_vel_y = np.random.uniform(-0.1, 0.1)
        self.ref_vel_ang = np.random.uniform(-0.1, 0.1)

        print("Reference velocity: ", self.ref_vel_x, self.ref_vel_y, self.ref_vel_ang)
        

        # export
        self.export_plot = export_plot
        self.tracking_duration = tracking_duration
        self.exported = False
        self.running_completed = False

        self.plotting = False
        self.joint_limit = None

    def _init_robot(self):
        self.robot = Robot(self.robot_type)
        if not self.robot.init(self.ip_addr):
            print("Failed to initialize robot with IP address: ", self.ip_addr)
            sys.exit(1)
        print("Robot initialized successfully.")
        self.joint_limit = self.robot.getJointLimit()
        print("Joint limit: ", self.joint_limit)




    def reset_saves(self):

        if self.exported:
            print("Data already exported. Resetting won't take effect.")
            return
        
        if self.running_completed:
            print("Tracking already completed. Resetting won't take effect.")
            return
        
        self.save_reset = True
        
        self.t_current = time.time()
        self.t_prev = self.t_current

        self.gt_imu_pos = [[] for _ in range(3)]
        self.gt_imu_vel = [[] for _ in range(3)]
        self.gt_imu_acc = [[] for _ in range(3)]
        self.gt_imu_ang = [[] for _ in range(3)] # euler angles
        self.gt_imu_ang_vel = [[] for _ in range(3)]
        self.gt_t = []


        self.imu_t_current = 0
        self.imu_t_prev = 0

        #self.imu_pos = [[] for _ in range(3)]
        self.imu_vel = [[] for _ in range(3)]
        self.imu_acc = [[] for _ in range(3)]
        self.imu_ang = [[] for _ in range(3)]
        self.imu_ang_vel = [[] for _ in range(3)]
        self.imu_t = []


        self.rs_t_current = 0
        self.rs_t_prev = 0
        self.rs_t = []
        self.left_vel = [[] for _ in range(3)]
        self.right_vel = [[] for _ in range(3)]
        self.left_q = [[] for _ in range(3)]
        self.right_q = [[] for _ in range(3)]
        self.left_dq = [[] for _ in range(3)]
        self.right_dq = [[] for _ in range(3)]
        self.estimated_vel = [[] for _ in range(3)]
        self.left_tau = [[] for _ in range(3)]
        self.right_tau = [[] for _ in range(3)]
        self.feet_state_left = [[] for _ in range(3)]
        self.feet_state_right = [[] for _ in range(3)]


    def reset_world(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            time.sleep(1)
            reset_world()
        
            self.world_reset = True
            self.reset_saves()
            self.world_reset_count += 1
            print("World reset successfully.")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def reset_world_prev(self):
        time.sleep(1)
        print("Resetting world...")
        self.reset_world()
        print("World reset thread finished. Starting tracking...")
        self.time_control_thread()

    def reset_world_thread(self):
        reset_thread = Thread(target=self.reset_world_prev)
        reset_thread.start()

    def clock_cb(self, data):
        clock_time = data.clock
        if clock_time < self.sim_time:
            print('Gazebo reset world detected')
            self.world_reset = True
            self.world_reset_count += 1
        
        self.sim_time = clock_time

    def quaternion_to_euler(self, q):
        # q = [w, x, y, z]
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def link_states_cb(self, data):
        if not self.world_reset and self.world_reset_count == 0:
            return
        
        if self.exported:
            return
        
        if self.running_completed:
            return
        
        imu_idx = data.name.index(self.gazebo_imu_name)
        imu_pos = data.pose[imu_idx].position
        imu_orientation = data.pose[imu_idx].orientation
        imu_twist = data.twist[imu_idx]
        imu_linear = imu_twist.linear
        imu_angular = imu_twist.angular

        self.gt_imu_pos[0].append(imu_pos.x)
        self.gt_imu_pos[1].append(imu_pos.y)
        self.gt_imu_pos[2].append(imu_pos.z)

        self.gt_imu_vel[0].append(imu_linear.x)
        self.gt_imu_vel[1].append(imu_linear.y)
        self.gt_imu_vel[2].append(imu_linear.z)

        self.gt_imu_ang[0].append(imu_orientation.x)
        self.gt_imu_ang[1].append(imu_orientation.y)
        self.gt_imu_ang[2].append(imu_orientation.z)

        self.gt_imu_ang_vel[0].append(imu_angular.x)
        self.gt_imu_ang_vel[1].append(imu_angular.y)
        self.gt_imu_ang_vel[2].append(imu_angular.z)
        t_current = time.time()

        if self.save_reset:
            self.save_reset = False
            self.gt_t.append(0)
            self.imu_t_prev = time.time()
            self.rs_t_prev = time.time()
            self.gt_imu_acc[0].append(0)
            self.gt_imu_acc[1].append(0)
            self.gt_imu_acc[2].append(0)
        else:
            dt = (t_current - self.t_prev)
            self.gt_t.append(self.gt_t[-1] + dt)
            if dt == 0:
                dt = 0.0001
            self.t_prev = t_current
            self.gt_imu_acc[0].append((self.gt_imu_vel[0][-1] - self.gt_imu_vel[0][-2]) / dt)
            self.gt_imu_acc[1].append((self.gt_imu_vel[1][-1] - self.gt_imu_vel[1][-2]) / dt)
            self.gt_imu_acc[2].append((self.gt_imu_vel[2][-1] - self.gt_imu_vel[2][-2]) / dt)


    def imu_cb(self, data: datatypes.ImuData):
        if not self.world_reset and self.world_reset_count == 0:
            return
        
        if self.exported:
            return
        
        if self.running_completed:
            return
        
        if self.imu_t_prev == 0: # not initialized
            return
        
        self.imu_t_current = time.time()
        dt = self.imu_t_current - self.imu_t_prev
        self.imu_t.append(dt) if len(self.imu_t) == 0 else self.imu_t.append(self.imu_t[-1] + dt)
        acc = data.acc
        gyro = data.gyro
        quat = data.quat
        quat = [quat[0], quat[1], quat[2], quat[3]]
        #print(quat)
        euler = self.quaternion_to_euler(quat)
        #print(euler)
        
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        # calculate rotation matrix
        R = np.array([
            [np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
            [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
            [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll)]
        ])

        acc = np.dot(R, acc)
        acc[2] -= 9.80655

        self.imu_acc[0].append(acc[0])
        self.imu_acc[1].append(acc[1])
        self.imu_acc[2].append(acc[2])
        # if len(self.imu_t) - 1 >= 2: # filter
        # #     self.imu_acc[0][-1] = savgol_filter(self.imu_acc[0][-3:], 3, 1)[-1]
        # #     self.imu_acc[1][-1] = savgol_filter(self.imu_acc[1][-3:], 3, 1)[-1]
        # #     self.imu_acc[2][-1] = savgol_filter(self.imu_acc[2][-3:], 3, 1)[-1]
        #     # b, a = butter(2, 0.1)
        #     # self.imu_acc[0][-1] = lfilter(b, a, self.imu_acc[0][-3:])[0]
        #     # self.imu_acc[1][-1] = lfilter(b, a, self.imu_acc[1][-3:])[0]
        #     # self.imu_acc[2][-1] = lfilter(b, a, self.imu_acc[2][-3:])[0]
        #     self.imu_acc[0][-1] = 0.75 * self.imu_acc[0][-2] + 0.25 * self.imu_acc[0][-1]
        #     self.imu_acc[1][-1] = 0.75 * self.imu_acc[1][-2] + 0.25 * self.imu_acc[1][-1]
        #     self.imu_acc[2][-1] = 0.75 * self.imu_acc[2][-2] + 0.25 * self.imu_acc[2][-1]


        self.imu_ang[0].append(roll)
        self.imu_ang[1].append(pitch)
        self.imu_ang[2].append(yaw)

        self.imu_ang_vel[0].append(gyro[0])
        self.imu_ang_vel[1].append(gyro[1])
        self.imu_ang_vel[2].append(gyro[2])

        # calculate velocity
        v_delta = np.array(acc) * dt
        if len(self.imu_vel[0]) == 0:
            self.imu_vel[0].append(v_delta[0])
            self.imu_vel[1].append(v_delta[1])
            self.imu_vel[2].append(v_delta[2])

        else:
            self.imu_vel[0].append(self.imu_vel[0][-1] + v_delta[0])
            self.imu_vel[1].append(self.imu_vel[1][-1] + v_delta[1])
            self.imu_vel[2].append(self.imu_vel[2][-1] + v_delta[2])

        self.imu_t_prev = self.imu_t_current

    def rs_cb(self, data: datatypes.RobotState):
        if not self.world_reset and self.world_reset_count == 0:
            return
        
        if self.exported:
            return
        
        if self.running_completed:
            return
        
        if self.imu_t_prev == 0: # not initialized
            return
        
        self.rs_t_current = time.time()
        dt = self.rs_t_current - self.rs_t_prev
        self.rs_t.append(dt) if len(self.rs_t) == 0 else self.rs_t.append(self.rs_t[-1] + dt)
        tau = data.tau
        q = data.q
        dq = data.dq
        left_q = q[:3]
        right_q = q[3:]
        dq_real = np.array(dq)
        left_vel = py_state_vel_track(q, dq_real, self.q_init, self.l, True)
        right_vel = py_state_vel_track(q, dq_real, self.q_init, self.l, False)
        try:
            feet_states = py_feet_distance_track(q, self.q_init, self.l)
            self.feet_state_left[0].append(feet_states[0])
            self.feet_state_left[1].append(feet_states[1])
            self.feet_state_left[2].append(feet_states[2])
            self.feet_state_right[0].append(feet_states[3] + self.feet_original_offset[0])
            self.feet_state_right[1].append(feet_states[4] + self.feet_original_offset[1])
            self.feet_state_right[2].append(feet_states[5] + self.feet_original_offset[2])

        except:
            pass



        self.left_vel[0].append(left_vel[0])
        self.left_vel[1].append(left_vel[1])
        self.left_vel[2].append(left_vel[2])

        self.right_vel[0].append(right_vel[0])
        self.right_vel[1].append(right_vel[1])
        self.right_vel[2].append(right_vel[2])

        self.left_tau[0].append(tau[0])
        self.left_tau[1].append(tau[1])
        self.left_tau[2].append(tau[2])

        self.right_tau[0].append(tau[3])
        self.right_tau[1].append(tau[4])
        self.right_tau[2].append(tau[5])

        self.left_q[0].append(left_q[0])
        self.left_q[1].append(left_q[1])
        self.left_q[2].append(left_q[2])

        self.right_q[0].append(right_q[0])
        self.right_q[1].append(right_q[1])
        self.right_q[2].append(right_q[2])

        self.left_dq[0].append(dq[0])
        self.left_dq[1].append(dq[1])
        self.left_dq[2].append(dq[2])

        self.right_dq[0].append(dq[3])
        self.right_dq[1].append(dq[4])
        self.right_dq[2].append(dq[5])

        if len(self.estimated_vel[0]) == 0:
            l_estimate = (self.q_init[2] + left_q[2]) > (self.q_init[5] - right_q[2]) 
        else:
            l_estimate = (left_vel[0] - self.estimated_vel[0][-1]) ** 2 + (left_vel[1] - self.estimated_vel[1][-1]) ** 2 + (left_vel[2] - self.estimated_vel[2][-1]) ** 2 < (right_vel[0] - self.estimated_vel[0][-1]) ** 2 + (right_vel[1] - self.estimated_vel[1][-1]) ** 2 + (right_vel[2] - self.estimated_vel[2][-1]) ** 2

        if l_estimate:
            self.estimated_vel[0].append(left_vel[0])
            self.estimated_vel[1].append(left_vel[1])
            self.estimated_vel[2].append(left_vel[2])

        else:
            self.estimated_vel[0].append(right_vel[0])
            self.estimated_vel[1].append(right_vel[1])
            self.estimated_vel[2].append(right_vel[2])

        
        
        self.rs_t_prev = self.rs_t_current

    def find_falling_time(self):
        # find falling time of robot by checking imu acceleration
        if len(self.imu_acc[2]) < 2:
            return -1
        # apply ma(3) filter for x, y, z acceleration
        acc_x = savgol_filter(self.imu_acc[0], 3, 1)
        acc_y = savgol_filter(self.imu_acc[1], 3, 1)
        acc_z = savgol_filter(self.imu_acc[2], 3, 1)

        # find the first time when sqrt(acc_x^2 + acc_y^2 + acc_z^2) > 100 * sqrt(3)
        for i in range(len(acc_x)):
            if np.sqrt(acc_x[i] ** 2 + acc_y[i] ** 2 + acc_z[i] ** 2) > 100 * np.sqrt(3):
                return i
            
        return -1

    def plot_pose(self):
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.gt_t, self.gt_imu_pos[0], label='gt')
        rs_dt = np.diff(self.rs_t)
        rs_dt = np.insert(rs_dt, 0, self.rs_t[0])
        int_esti_x = np.cumsum(self.estimated_vel[0] * rs_dt)
        ax[0].plot(self.rs_t, int_esti_x, label='estimated')
        ax[0].set_title("X Position")
        ax[0].legend()
        ax[1].plot(self.gt_t, self.gt_imu_pos[1], label='gt')
        int_esti_y = np.cumsum(self.estimated_vel[1] * rs_dt)
        ax[1].plot(self.rs_t, int_esti_y, label='estimated')
        ax[1].set_title("Y Position")
        ax[1].legend()
        ax[2].plot(self.gt_t, self.gt_imu_pos[2], label='gt')
        int_esti_z = np.cumsum(self.estimated_vel[2] * rs_dt)
        ax[2].plot(self.rs_t, int_esti_z, label='estimated')
        ax[2].set_title("Z Position")
        ax[2].legend()
        plt.savefig(os.path.join(self.export_path, "gt_imu_pos.png"))

    def plot_velocity(self):
        # plot gt and imu velocity
        falling_time_idx = self.find_falling_time()
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.gt_t, self.gt_imu_vel[0], label='gt')
        ax[0].plot(self.imu_t, self.imu_vel[0], label='imu')
        ax[0].plot(self.gt_t, self.ref_vel_x * np.ones(len(self.gt_t)), label='ref')
        if falling_time_idx != -1:
            ax[0].axvline(x=self.gt_t[falling_time_idx], color='r', linestyle='--', label='falling time')
        ax[0].set_title("X Velocity")
        ax[0].legend()
        ax[1].plot(self.gt_t, self.gt_imu_vel[1], label='gt')
        ax[1].plot(self.imu_t, self.imu_vel[1], label='imu')
        ax[1].plot(self.gt_t, self.ref_vel_y * np.ones(len(self.gt_t)), label='ref')
        ax[1].set_title("Y Velocity")
        if falling_time_idx != -1:
            ax[1].axvline(x=self.gt_t[falling_time_idx], color='r', linestyle='--', label='falling time')
        ax[1].legend()
        ax[2].plot(self.gt_t, self.gt_imu_vel[2], label='gt')
        ax[2].plot(self.imu_t, self.imu_vel[2], label='imu')
        ax[2].plot(self.gt_t, np.zeros(len(self.gt_t)), label='ref')
        ax[2].set_title("Z Velocity")
        if falling_time_idx != -1:
            ax[2].axvline(x=self.gt_t[falling_time_idx], color='r', linestyle='--', label='falling time')

        ax[2].legend()
        plt.savefig(os.path.join(self.export_path, "gt_imu_vel.png"))

        

    def plot_acceleration(self):
        # plot gt and imu acceleration
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.gt_t, self.gt_imu_acc[0], label='gt')
        ax[0].plot(self.imu_t, self.imu_acc[0], label='imu')
        ax[0].set_title("X Acceleration")
        ax[0].legend()
        ax[1].plot(self.gt_t, self.gt_imu_acc[1], label='gt')
        ax[1].plot(self.imu_t, self.imu_acc[1], label='imu')
        ax[1].set_title("Y Acceleration")
        ax[1].legend()
        ax[2].plot(self.gt_t, self.gt_imu_acc[2], label='gt')
        ax[2].plot(self.imu_t, self.imu_acc[2], label='imu')
        ax[2].set_title("Z Acceleration")
        ax[2].legend()

        plt.savefig(os.path.join(self.export_path, "gt_imu_acc.png"))
        


    def plot_angular_velocity(self):
        # plot gt and imu angular velocity
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.gt_t, self.gt_imu_ang_vel[0], label='gt')
        ax[0].plot(self.imu_t, self.imu_ang_vel[0], label='imu')
        # plot 0 as reference x
        ax[0].plot(self.gt_t, np.zeros(len(self.gt_t)), label='ref')
        ax[0].set_title("X Angular Velocity")
        ax[0].legend()
        ax[1].plot(self.gt_t, self.gt_imu_ang_vel[1], label='gt')
        ax[1].plot(self.imu_t, self.imu_ang_vel[1], label='imu')
        # plot 0 as reference y
        ax[1].plot(self.gt_t, np.zeros(len(self.gt_t)), label='ref')
        ax[1].set_title("Y Angular Velocity")
        ax[1].legend()
        ax[2].plot(self.gt_t, self.gt_imu_ang_vel[2], label='gt')
        ax[2].plot(self.imu_t, self.imu_ang_vel[2], label='imu')
        # plot self.ref_vel_ang as reference z
        ax[2].plot(self.gt_t, np.ones(len(self.gt_t)) * self.ref_vel_ang, label='ref')
        ax[2].set_title("Z Angular Velocity")
        ax[2].legend()

        plt.savefig(os.path.join(self.export_path , "gt_imu_ang_vel.png"))
        


    def plot_euler_angles(self):
        # plot gt and imu euler angles
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.gt_t, self.gt_imu_ang[0], label='gt')
        ax[0].plot(self.imu_t, self.imu_ang[0], label='imu')
        ax[0].set_title("Roll")
        ax[0].legend()
        ax[1].plot(self.gt_t, self.gt_imu_ang[1], label='gt')
        ax[1].plot(self.imu_t, self.imu_ang[1], label='imu')
        ax[1].set_title("Pitch")
        ax[1].legend()
        ax[2].plot(self.gt_t, self.gt_imu_ang[2], label='gt')
        ax[2].plot(self.imu_t, self.imu_ang[2], label='imu')
        ax[2].set_title("Yaw")
        ax[2].legend()

        plt.savefig(os.path.join(self.export_path, "gt_imu_ang.png"))

    def plot_state_vel(self):
        # plot left and right leg velocity, with imu velocity as ground truth
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.rs_t, self.left_vel[0], label='left')
        ax[0].plot(self.rs_t, self.right_vel[0], label='right')
        #ax[0].plot(self.rs_t, self.estimated_vel[0], label='estimated')
        ax[0].plot(self.gt_t, self.gt_imu_vel[0], label='gt')
        ax[0].set_title("X Velocity")
        ax[0].legend()
        
        ax[1].plot(self.rs_t, self.left_vel[1], label='left')
        ax[1].plot(self.rs_t, self.right_vel[1], label='right')
        #ax[1].plot(self.rs_t, self.estimated_vel[1], label='estimated')
        ax[1].plot(self.gt_t, self.gt_imu_vel[1], label='gt')
        ax[1].set_title("Y Velocity")
        ax[1].legend()

        ax[2].plot(self.rs_t, self.left_vel[2], label='left')
        ax[2].plot(self.rs_t, self.right_vel[2], label='right')
        #ax[2].plot(self.rs_t, self.estimated_vel[2], label='estimated')
        ax[2].plot(self.gt_t, self.gt_imu_vel[2], label='gt')
        ax[2].set_title("Z Velocity")
        ax[2].legend()

        plt.savefig(os.path.join(self.export_path, "state_vel.png"))

    def plot_state_angl(self):
        # plot left and right q and int of dq
        fig, ax = plt.subplots(6, 1, figsize=(12, 12))
        int_left_dq = np.cumsum(self.left_dq[0]) + self.q_init[0]
        ax[0].plot(self.rs_t, self.left_q[0], label='left')
        #ax[0].plot(self.rs_t, int_left_dq, label='integrated')
        ax[0].set_title("Left Abad")
        ax[0].legend()

        int_left_dq = np.cumsum(self.left_dq[1]) + self.q_init[1]
        ax[1].plot(self.rs_t, self.left_q[1], label='left')
        #ax[1].plot(self.rs_t, int_left_dq, label='integrated')
        ax[1].set_title("Left Knee")
        ax[1].legend()

        int_left_dq = np.cumsum(self.left_dq[2]) + self.q_init[2]
        ax[2].plot(self.rs_t, self.left_q[2], label='left')
        #ax[2].plot(self.rs_t, int_left_dq, label='integrated')
        ax[2].set_title("Left Ankle")
        ax[2].legend()

        int_right_dq = np.cumsum(self.right_dq[0]) + self.q_init[3]
        ax[3].plot(self.rs_t, self.right_q[0], label='right')
        #ax[3].plot(self.rs_t, int_right_dq, label='integrated')
        ax[3].set_title("Right Abad")
        ax[3].legend()

        int_right_dq = np.cumsum(self.right_dq[1]) + self.q_init[4]
        ax[4].plot(self.rs_t, self.right_q[1], label='right')
        #ax[4].plot(self.rs_t, int_right_dq, label='integrated')
        ax[4].set_title("Right Knee")

        int_right_dq = np.cumsum(self.right_dq[2]) + self.q_init[5]
        ax[5].plot(self.rs_t, self.right_q[2], label='right')
        #ax[5].plot(self.rs_t, int_right_dq, label='integrated')
        ax[5].set_title("Right Ankle")
        ax[5].legend()

        plt.savefig(os.path.join(self.export_path ,"state_angl.png"))

    def plot_state_dq(self):
        fig, ax = plt.subplots(6, 1, figsize=(12, 12))
        ax[0].plot(self.rs_t, self.left_dq[0], label='left')
        ax[0].set_title("Left Abad")
        ax[0].legend()

        ax[1].plot(self.rs_t, self.left_dq[1], label='left')
        ax[1].set_title("Left Knee")
        ax[1].legend()

        ax[2].plot(self.rs_t, self.left_dq[2], label='left')
        ax[2].set_title("Left Ankle")
        ax[2].legend()

        ax[3].plot(self.rs_t, self.right_dq[0], label='right')
        ax[3].set_title("Right Abad")
        ax[3].legend()

        ax[4].plot(self.rs_t, self.right_dq[1], label='right')
        ax[4].set_title("Right Knee")
        ax[4].legend()

        ax[5].plot(self.rs_t, self.right_dq[2], label='right')
        ax[5].set_title("Right Ankle")
        ax[5].legend()

        plt.savefig(os.path.join(self.export_path, "state_dq.png"))

    def plot_feet_state(self):
        fig, ax = plt.subplots(3, 1, figsize=(12, 12))
        ax[0].plot(self.rs_t, self.feet_state_left[0], label='left')
        ax[0].plot(self.rs_t, self.feet_state_right[0], label='right')
        ax[0].set_title("X Feet State")
        ax[0].legend()

        ax[1].plot(self.rs_t, self.feet_state_left[1], label='left')
        ax[1].plot(self.rs_t, self.feet_state_right[1], label='right')
        ax[1].set_title("Y Feet State")
        ax[1].legend()

        ax[2].plot(self.rs_t, self.feet_state_left[2], label='left')
        ax[2].plot(self.rs_t, self.feet_state_right[2], label='right')
        ax[2].set_title("Z Feet State")
        ax[2].legend()

        plt.savefig(os.path.join(self.export_path, "feet_state.png"))


    
    
    def export_data(self):
        # export data as npz
        export_path = os.path.join(self.export_path, "exported_data.npz")
        t = np.array(self.imu_t)
        base_lin_vel = np.array(self.imu_vel).swapaxes(-1, -2)
        base_ang_vel = np.array(self.imu_ang_vel).swapaxes(-1, -2)
        torques = np.array(self.left_tau + self.right_tau).swapaxes(-1, -2)
        torque_limits = -0.1
        q = np.array(self.left_q + self.right_q).swapaxes(-1, -2)
        last_q = np.concatenate([np.zeros((1, 6)), q[:-1]], axis=0)
        dq = np.array(self.left_dq + self.right_dq).swapaxes(-1, -2)
        last_dq = np.concatenate([np.zeros((1, 6)), dq[:-1]], axis=0)
        ddq = np.diff(dq, axis=0) / np.diff(self.rs_t)[:,np.newaxis]
        commands = np.array([self.ref_vel_x, self.ref_vel_y, self.ref_vel_ang]).repeat(len(t), axis=0).reshape(-1, 3)
        survival_time = t[self.find_falling_time()] # = t[-1] if no falling time detected
        survival_idx = self.find_falling_time()
        left_feet_state = np.array(self.feet_state_left).swapaxes(-1, -2)
        right_feet_state = np.array(self.feet_state_right).swapaxes(-1, -2)
        feet_state = np.array([left_feet_state, right_feet_state]).swapaxes(0, 1)
        left_feet_height = np.array(self.feet_state_left[2])
        right_feet_height = np.array(self.feet_state_right[2])  
        feet_height = np.array([left_feet_height, right_feet_height]).swapaxes(0, 1)
        last_feet_height = np.concatenate([np.zeros((1, 2)), feet_height[:-1]], axis=0)

        np.savez(export_path, t=t, base_lin_vel=base_lin_vel, base_ang_vel=base_ang_vel, torques=torques, torque_limits = torque_limits, q=q, last_q=last_q, dq=dq, last_dq=last_dq, ddq=ddq, feet_state=feet_state, commands=commands, survival_time=survival_time, survival_idx=survival_idx, feet_height=feet_height, last_feet_height=last_feet_height)


        # np.savez(export_path,
        #             # gt_imu_pos=np.array(self.gt_imu_pos),
        #             # gt_imu_vel=np.array(self.gt_imu_vel),
        #             # gt_imu_acc=np.array(self.gt_imu_acc),
        #             # gt_imu_ang=np.array(self.gt_imu_ang),
        #             # gt_imu_ang_vel=np.array(self.gt_imu_ang_vel),
        #             # gt_t=np.array(self.gt_t),
        #             imu_vel=np.array(self.imu_vel),
        #             imu_acc=np.array(self.imu_acc),
        #             imu_ang=np.array(self.imu_ang),
        #             imu_ang_vel=np.array(self.imu_ang_vel),
        #             imu_t=np.array(self.imu_t),
        #             left_vel=np.array(self.left_vel),
        #             right_vel=np.array(self.right_vel),
        #             left_q=np.array(self.left_q),
        #             right_q=np.array(self.right_q),
        #             left_dq=np.array(self.left_dq),
        #             right_dq=np.array(self.right_dq),
        #             rs_t=np.array(self.rs_t),
        #             estimated_vel=np.array(self.estimated_vel),
        #             left_tau=np.array(self.left_tau),
        #             right_tau=np.array(self.right_tau),
        #             falling_time_idx=self.find_falling_time(),
        #             ref_vel_x=self.ref_vel_x,
        #             ref_vel_y=self.ref_vel_y,
        #             ref_vel_ang=self.ref_vel_ang,
        #             )
        print("Data exported to ", os.path.join(os.getcwd(), export_path))


    def time_control(self):
        t_start = time.time()
        self.plotting = True
        while time.time() - t_start < self.tracking_duration:
            time.sleep(1)
        rospy.signal_shutdown("Tracking completed.")
        print("Tracking completed.")
        self.plotting = False

    def time_control_thread(self):
        time_thread = Thread(target=self.time_control)
        time_thread.start()

    def run_gazebo(self):
        # set logging directory
        log_dir = os.path.join(self.export_path, "gazebo_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        with open(os.path.join(log_dir, "gazebo.log"), "w") as out:
            # 1. remove workspace install folder
            subprocess.run(["rm", "-rf", os.path.join(self.project_path, "install")], stdout=out, stderr=out)
            time.sleep(1)
            # 2. execute gazebo running script (under project path)
            # get project path
            cur_file_path = os.path.abspath(__file__)
            cur_dir = os.path.dirname(cur_file_path)
            run_gazebo = subprocess.Popen([os.path.join(cur_dir, "scripts", "make_and_run.sh")], stdout=out, stderr=out, preexec_fn=os.setsid)
            print("Gazebo started. Running with PID: ", run_gazebo.pid)

            while not self.exported:
                time.sleep(1)

            # 3. terminate gazebo
            os.killpg(os.getpgid(run_gazebo.pid), signal.SIGTERM)

            run_gazebo.wait()
            print("Gazebo terminated.")

    def speed_control_loop(self):
        # publish /cmd_vel
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        while not self.exported:
            if not self.world_reset and self.world_reset_count == 0:
                continue
            twist = Twist()
            twist.linear.x = self.ref_vel_x
            twist.linear.y = self.ref_vel_y
            twist.angular.z = self.ref_vel_ang
            pub.publish(twist)
            rate.sleep()

    def run(self):

        gazebo_thread = Thread(target=self.run_gazebo)
        gazebo_thread.start()

        print("Tracking started. Waiting for gazebo to start...")
        time.sleep(10)
        self._init_robot()


        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb)
        imu_cb = partial(self.imu_cb)
        self.robot.subscribeImuData(imu_cb)
        rs_cb = partial(self.rs_cb)
        self.robot.subscribeRobotState(rs_cb)

        reset_thread = Thread(target=self.reset_world_thread)
        reset_thread.start()
        rospy.spin() # keep the node running until rospy.shutdown() is called
        self.running_completed = True
        self.export_data()
        self.exported = True
        print("Tracking completed.")
        
        # self.plot_pose_thread.join()
        # self.plot_velocity_thread.join()
        # self.plot_acceleration_thread.join()
        # self.plot_angular_velocity_thread.join()
        # self.plot_euler_angles_thread.join()

        global_path = os.getcwd()

        if self.export_plot:
            self.plot_pose()
            self.plot_velocity()
            self.plot_acceleration()
            self.plot_angular_velocity()
            self.plot_euler_angles()
            self.plot_state_vel()
            self.plot_state_angl()
            self.plot_state_dq()
            try:
                self.plot_feet_state()
            except:
                print("No feet state data available.")
            print("Plots exported to ", os.path.join(global_path, self.export_path))

        falling_time = self.find_falling_time()
        if falling_time != -1:
            print("Falling time detected at ", self.imu_t[falling_time])
        else:
            print("No falling time detected.")

        gazebo_thread.join()
        reset_thread.join()



if __name__ == "__main__":

    # argparser
    parser = argparse.ArgumentParser()
    parser.add_argument("--project_path", type=str, default="")
    parser.add_argument("--ip_addr", type=str, default="127.0.0.1")
    parser.add_argument("--export_path", type=str, default="exported_data/")
    parser.add_argument("--export_plot", type=bool, default=True)
    parser.add_argument("--tracking_duration", type=int, default=30)

    args = parser.parse_args()


    tracker = SimEnvRobotTracker(
        ip_addr=args.ip_addr,
        project_path=args.project_path,
        export_path=args.export_path,
        export_plot=args.export_plot,
        tracking_duration=args.tracking_duration
        
    )
    tracker.run()
