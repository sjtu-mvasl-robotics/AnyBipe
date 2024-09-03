/**
 * @file pf_controller_base.h
 * @brief Header file for the PFControllerBase class, which serves as the base class for robot controllers.
 * @version 1.0
 * @date 2024-3-5
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include <thread>              // Include for std::thread
#include <mutex>               // Include for std::mutex
#include <vector>              // Include for std::vector
#include "limxsdk/pointfoot.h"// Include for limxsdk::PointFoot
#include <Eigen/Dense>         // Include for Eigen library (dense matrix algebra)
#include <iostream>            // Include for standard input/output operations
#include "unistd.h"            // Include for usleep function (Unix standard)
#ifdef WIN32
#include <windows.h>           // Include for Windows-specific functions
#pragma comment(lib, "winmm.lib")
#endif

/**
 * @brief Controller base class for robot controllers
 */
class PFControllerBase
{
public:
  /**
   * @brief Constructor of the PFControllerBase class.
   */
  PFControllerBase();

  /**
   * @brief Destructor of the PFControllerBase class.
   */
  ~PFControllerBase();

protected:
  /**
   * @brief Function to control a single joint of the robot.
   *
   * @param jointId The ID of the joint to control.
   * @param kp Proportional gain for the PID controller.
   * @param kd Derivative gain for the PID controller.
   * @param targetPos Target position for the joint.
   * @param targetVel Target velocity for the joint (currently not used).
   * @param targetTorque Target torque for the joint (currently not used).
   */
  void singleJointController(int jointId, double kp, double kd,
                             double targetPos, double targetVel,
                             double targetTorque);

  /**
   * @brief Function to control all joints of the robot simultaneously.
   *
   * @param kp Vector containing proportional gains for all joints.
   * @param kd Vector containing derivative gains for all joints.
   * @param targetPos Vector containing target positions for all joints.
   * @param targetVel Vector containing target velocities for all joints (currently not used).
   * @param targetTorque Vector containing target torques for all joints (currently not used).
   */
  void groupJointController(std::vector<float> &kp, std::vector<float> &kd,
                            std::vector<float> &targetPos, std::vector<float> &targetVel,
                            std::vector<float> &targetTorque);

  /**
   * @brief Function to publish zero torque commands to all joints.
   */
  void zeroTorque();

  /**
   * @brief Function to publish damping commands to all joints.
   */
  void damping();

  /**
   * @brief Function to get the number of joints in the robot.
   *
   * @return The number of joints.
   */
  int getNumofJoint() { return joint_num_; }

  const int32_t ROBOT_CMD_RATE = 1000; // Rate of robot command updates in milliseconds

  std::mutex mtx_;             // Mutex for thread safety

  limxsdk::PointFoot *pf_;     // Pointer to the PointFoot instance
  limxsdk::RobotCmd robot_cmd_;// Robot command object
  limxsdk::RobotState robot_state_; // Robot state object
  limxsdk::ImuData imu_data_; // Imu data object

  bool robotstate_on_;         // Flag indicating if robot state is received
  bool is_first_enter_{true};  // Flag indicating the first iteration
  double time_start_{0.0};     // Start time for an action
  double time_action_ = 3.0;   // Duration of an action
  int running_iter_{1};        // Iteration count

  Eigen::VectorXd joint_offset_; // Vector containing joint offsets
  Eigen::VectorXd joint_limit_;  // Vector containing joint limits

private:
  int joint_num_{6};           // Number of joints in the robot
};
