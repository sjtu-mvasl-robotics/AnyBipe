/**
 * @file HybridJointInterface.h
 *
 * @brief This file contains the definition of the HybridJointHandle class and the HybridJointInterface class.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace pointfoot_gazebo
{
  /**
   * @brief A handle class for hybrid joints, which extends the JointStateHandle class.
   */
  class HybridJointHandle : public hardware_interface::JointStateHandle
  {
  public:
    /**
     * @brief Default constructor.
     */
    HybridJointHandle() = default;

    /**
     * @brief Constructor that initializes the HybridJointHandle with the provided data pointers for position desired, velocity desired, Kp (proportional gain), Kd (derivative gain), and feedforward.
     *
     * @param js The JointStateHandle to initialize the HybridJointHandle with.
     * @param posDes Pointer to the position desired data.
     * @param velDes Pointer to the velocity desired data.
     * @param kp Pointer to the Kp data.
     * @param kd Pointer to the Kd data.
     * @param ff Pointer to the feedforward data.
     *
     * @throw hardware_interface::HardwareInterfaceException If any of the data pointers is null.
     */
    HybridJointHandle(const JointStateHandle &js, double *posDes, double *velDes, double *kp, double *kd, double *ff)
        : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff)
    {
      if (posDes_ == nullptr)
      {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                             "'. Position desired data pointer is null.");
      }
      if (velDes_ == nullptr)
      {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                             "'. Velocity desired data pointer is null.");
      }
      if (kp_ == nullptr)
      {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
      }
      if (kd_ == nullptr)
      {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
      }
      if (ff_ == nullptr)
      {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                             "'. Feedforward data pointer is null.");
      }
    }

    /**
     * @brief Sets the position desired value.
     *
     * @param cmd The position desired value to set.
     */
    void setPositionDesired(double cmd)
    {
      assert(posDes_);
      *posDes_ = cmd;
    }

    /**
     * @brief Sets the velocity desired value.
     *
     * @param cmd The velocity desired value to set.
     */
    void setVelocityDesired(double cmd)
    {
      assert(velDes_);
      *velDes_ = cmd;
    }

    /**
     * @brief Sets the Kp (proportional gain) value.
     *
     * @param cmd The Kp value to set.
     */
    void setKp(double cmd)
    {
      assert(kp_);
      *kp_ = cmd;
    }

    /**
     * @brief Sets the Kd (derivative gain) value.
     *
     * @param cmd The Kd value to set.
     */
    void setKd(double cmd)
    {
      assert(kd_);
      *kd_ = cmd;
    }

    /**
     * @brief Sets the feedforward value.
     *
     * @param cmd The feedforward value to set.
     */
    void setFeedforward(double cmd)
    {
      assert(ff_);
      *ff_ = cmd;
    }

    /**
     * @brief Sets the position desired, velocity desired, Kp (proportional gain), Kd (derivative gain), and feedforward values.
     *
     * @param pos_des The position desired value to set.
     * @param vel_des The velocity desired value to set.
     * @param kp The Kp (proportional gain) value to set.
     * @param kd The Kd (derivative gain) value to set.
     * @param ff The feedforward value to set.
     */
    void setCommand(double pos_des, double vel_des, double kp, double kd, double ff)
    {
      setPositionDesired(pos_des);
      setVelocityDesired(vel_des);
      setKp(kp);
      setKd(kd);
      setFeedforward(ff);
    }

    /**
     * @return The desired position value.
     */
    double getPositionDesired()
    {
      assert(posDes_);
      return *posDes_;
    }

    /**
     * @return The desired velocity value.
     */
    double getVelocityDesired()
    {
      assert(velDes_);
      return *velDes_;
    }

    /**
     * @return The proportional gain (Kp) value.
     */
    double getKp()
    {
      assert(kp_);
      return *kp_;
    }

    /**
     * @return The derivative gain (Kd) value.
     */
    double getKd()
    {
      assert(kd_);
      return *kd_;
    }

    /**
     * @return The feedforward value.
     */
    double getFeedforward()
    {
      assert(ff_);
      return *ff_;
    }

  private:
    double *posDes_ = {nullptr}; /**< Pointer to the desired position value. */
    double *velDes_ = {nullptr}; /**< Pointer to the desired velocity value. */
    double *kp_ = {nullptr};     /**< Pointer to the proportional gain (Kp) value. */
    double *kd_ = {nullptr};     /**< Pointer to the derivative gain (Kd) value. */
    double *ff_ = {nullptr};     /**< Pointer to the feedforward value. */
  };

  /**
   * @brief A hardware interface for hybrid joints, which manages resources for `HybridJointHandle` objects.
   */
  class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources>
  {
  };

} // namespace pointfoot_gazebo
