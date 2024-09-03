﻿/**
 * @file apibase.h
 *
 * @brief This file contains the declarations of interface related to the control of robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_INTERFACE_H_
#define _LIMX_SDK_INTERFACE_H_

#include <string>
#include <functional>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"

namespace limxsdk {
  /**
   * @brief Base class for the LIMX SDK API.
   *        Provides interface methods for controlling and accessing robot functionality.
   */
  class LIMX_SDK_API ApiBase {
    public:
      ApiBase();
      
      virtual ~ApiBase();

      /**
       * @brief Virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set to "127.0.0.1",
       *                         while for a real robot, it may be set to "10.192.1.2".
       * @return True if init successfully, otherwise false.
       */
      virtual bool init(const std::string& robot_ip_address);

      /**
       * @brief Virtual method to get the number of motors in the robot.
       * @return The total number of motors.
       */
      virtual uint32_t getMotorNumber();

      /**
       * @brief Virtual method to subscribe to updates of the robot's IMU (Inertial Measurement Unit) data.
       * @param cb The callback function to be invoked when new IMU data is received.
       */
      virtual void subscribeImuData(std::function<void(const ImuDataConstPtr&)> cb);

      /**
       * @brief Virtual method to subscribe to robot state updates.
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      virtual void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb);

      /**
       * @brief Virtual method to publish a command to control the robot's actions.
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      virtual bool publishRobotCmd(const RobotCmd& cmd);

      /**
       * @brief Virtual method to subscribe to robot commands for simulation.
       * @param cb The callback function to be invoked when a robot command is received in simulation mode.
       */
      virtual void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr&)> cb);

      /**
       * @brief Virtual method to publish the robot state for simulation.
       * @param state The RobotState object representing the current state of the robot in simulation.
       * @return True if the state was successfully published, otherwise false.
       */
      virtual bool publishRobotStateForSim(const RobotState& state);

      /**
       * @brief Virtual method to publish IMU (Inertial Measurement Unit) data to the motion control algorithm for simulation.
       * The order of IMU data is as follows:
       *        Accelerometer: imu.acc
       *        Gyroscope: imu.gyro
       *        Quaternion: imu.quat
       * 
       * @param imu The IMU data to be published.
       * @return True if publishing is successful, false otherwise.
       */
      virtual bool publishImuDataForSim(const ImuData& imu);

      /**
       * @brief Virtual method to get the joint offset of the robot.
       * @param joint_offset A vector of floats where the joint offsets will be stored.
       * @param timeout Timeout value in seconds for getting the joint offset. Use -1 for infinite waiting time.
       * @return True if the joint offset was successfully retrieved, otherwise false.
       */
      virtual bool getJointOffset(std::vector<float>& joint_offset, float timeout = -1 /*seconds*/);

      /**
       * @brief Virtual method to get the joint limit of the robot.
       * @param joint_limit A vector of floats where the joint limits will be stored.
       * @param timeout Timeout value in seconds for getting the joint limit. Use -1 for infinite waiting time.
       * @return True if the joint limit was successfully retrieved, otherwise false.
       */
      virtual bool getJointLimit(std::vector<float>& joint_limit, float timeout = -1 /*seconds*/);

      /**
       * @brief Virtual method to subscribe to sensor inputs related to a joystick from the robot.
       * @param cb The callback function to be invoked when sensor input from a joystick is received from the robot.
       */
      virtual void subscribeSensorJoy(std::function<void(const SensorJoyConstPtr&)> cb);

      /**
       * @brief Virtual method to subscribe to diagnostic values from the robot.
       * 
       * Examples:
       * | name        | level  | code | msg
       * |-------------|--------|------|--------------------
       * | imu         | OK     | 0    | - IMU is functioning properly.
       * | imu         | ERROR  | -1   | - Error in IMU.
       * |-------------|--------|------|--------------------
       * | ethercat    | OK     | 0    | - EtherCAT is working fine.
       * | ethercat    | ERROR  | -1   | - EtherCAT error.
       * |-------------|--------|------|--------------------
       * | calibration | OK     | 0    | - Robot calibration successful.
       * | calibration | WARN   | 1    | - Robot calibration in progress.
       * | calibration | ERROR  | -1   | - Robot calibration failed.
       * |-------------|--------|------|--------------------
       * 
       * @param cb he callback function to be invoked when diagnostic values are received from the robot.
       */
      virtual void subscribeDiagnosticValue(std::function<void(const DiagnosticValueConstPtr&)> cb);

    protected:
      std::function<void(const ImuDataConstPtr&)> imu_data_callback_{nullptr};  // Callback function for handling IMU data updates.
      std::function<void(const RobotStateConstPtr&)> robot_state_callback_{nullptr};  // Callback function for handling robot state updates.
      std::function<void(const RobotCmdConstPtr&)> robot_cmd_callback_{nullptr};  // Callback function for handling robot commands in simulation mode.
      std::function<void(const SensorJoyConstPtr&)> sensor_joy_callback_{nullptr}; // Callback for handling joystick sensor inputs from the robot.
      std::function<void(const DiagnosticValueConstPtr&)> diagnostic_callback_{nullptr}; // Callback for handling diagnostic values from the robot.
  };
}

#endif