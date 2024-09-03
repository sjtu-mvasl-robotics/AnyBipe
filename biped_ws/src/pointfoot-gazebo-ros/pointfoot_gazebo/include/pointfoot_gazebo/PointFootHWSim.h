/**
 * @file PointFootHWSim.h
 *
 * @brief This file defines the PointFootHWSim class, which is responsible for simulating a legged robot.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <hardware_interface/ContactSensorInterface.h>
#include <hardware_interface/HybridJointInterface.h>
#include <realtime_tools/realtime_buffer.h>
#include "limxsdk/pointfoot_sim.h"

namespace pointfoot_gazebo
{
  /**
   * @brief Structure representing hybrid joint data.
   */
  struct HybridJointData
  {
    hardware_interface::JointHandle joint_; // Joint handle
    double posDes_{};                      // Desired position
    double velDes_{};                      // Desired velocity
    double kp_{};                          // Proportional gain
    double kd_{};                          // Derivative gain
    double ff_{};                          // Feedforward term
  };

  /**
   * @brief Structure representing hybrid joint command.
   */
  struct HybridJointCommand
  {
    ros::Time stamp_;   // Timestamp
    double posDes_{};   // Desired position
    double velDes_{};   // Desired velocity
    double kp_{};       // Proportional gain
    double kd_{};       // Derivative gain
    double ff_{};       // Feedforward term
  };

  /**
   * @brief Structure representing IMU data.
   */
  struct ImuData
  {
    gazebo::physics::LinkPtr linkPtr_; // Link pointer
    double ori_[4];                    // Orientation quaternion (xyzw)
    double oriCov_[9];                 // Orientation covariance matrix
    double angularVel_[3];             // Angular velocity (xyz)
    double angularVelCov_[9];          // Angular velocity covariance matrix
    double linearAcc_[3];              // Linear acceleration (xyz)
    double linearAccCov_[9];           // Linear acceleration covariance matrix
  };

  /**
   * @brief Class representing the PointFootHWSim simulation.
   */
  class PointFootHWSim : public gazebo_ros_control::DefaultRobotHWSim
  {
  public:
    /**
     * @brief Initialize the simulation.
     * @param robot_namespace The namespace of the robot.
     * @param model_nh The node handle for the model.
     * @param parent_model The pointer to the parent model.
     * @param urdf_model The URDF model of the robot.
     * @param transmissions Vector of transmission information.
     * @return True if initialization is successful, false otherwise.
     */
    bool initSim(const std::string &robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                 const urdf::Model *urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;

    /**
     * @brief Read data from the simulation.
     * @param time Current time.
     * @param period Time step duration.
     */
    void readSim(ros::Time time, ros::Duration period) override;

    /**
     * @brief Write commands to the simulation.
     * @param time Current time.
     * @param period Time step duration.
     */
    void writeSim(ros::Time time, ros::Duration period) override;

  private:
    /**
     * @brief Parse IMU data from XML-RPC value.
     * @param imuDatas The XML-RPC value containing IMU data.
     * @param parentModel The pointer to the parent model.
     */
    void parseImu(XmlRpc::XmlRpcValue &imuDatas, const gazebo::physics::ModelPtr &parentModel);

    /**
     * @brief Parse contact names from XML-RPC value.
     * @param contactNames The XML-RPC value containing contact names.
     */
    void parseContacts(XmlRpc::XmlRpcValue &contactNames);

    /**
     * @brief Parse joint index for a given joint name.
     * @param jointName The name of the joint.
     * @return The index of the joint.
     */
    int parseJointIndex(const std::string& jointName);

    HybridJointInterface hybridJointInterface_;   // Hybrid joint interface
    ContactSensorInterface contactSensorInterface_; // Contact sensor interface
    hardware_interface::ImuSensorInterface imuSensorInterface_; // IMU sensor interface

    gazebo::physics::ContactManager *contactManager_{}; // Pointer to the contact manager

    std::list<HybridJointData> hybridJointDatas_; // List of hybrid joint data

    std::list<ImuData> imuDatas_;  // List of IMU data

    std::unordered_map<std::string, std::deque<HybridJointCommand>> cmdBuffer_; // Command buffer for each joint

    std::unordered_map<std::string, bool> name2contact_; // Mapping from contact names to boolean values indicating contact status

    realtime_tools::RealtimeBuffer<limxsdk::RobotCmd> robotCmdBuffer_; // Real-time buffer for robot commands

    double delay_{};  // Delay parameter for the simulation
  };

} // namespace pointfoot_gazebo
