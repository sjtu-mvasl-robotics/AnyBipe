/**
 * @file ContactSensorInterface.h
 *
 * @brief This file defines the ContactSensorInterface class and the ContactSensorHandle class.
 *        The ContactSensorInterface class is a hardware interface that manages contact sensor resources.
 *        The ContactSensorHandle class represents a handle to a specific contact sensor, providing methods for accessing its properties.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace pointfoot_gazebo
{
    /**
     * @brief The ContactSensorHandle class represents a handle to a specific contact sensor.
     *        It provides methods for accessing the name and contact status of the sensor.
     */
    class ContactSensorHandle
    {
    public:
        /**
         * @brief Default constructor for ContactSensorHandle.
         */
        ContactSensorHandle() = default;

        /**
         * @brief Constructor for ContactSensorHandle.
         * @param name The name of the contact sensor.
         * @param isContact A pointer to a boolean indicating the contact status of the sensor.
         * @throw hardware_interface::HardwareInterfaceException if the isContact pointer is null.
         */
        ContactSensorHandle(const std::string &name, const bool *isContact) : name_(name), isContact_(isContact)
        {
            if (isContact == nullptr)
            {
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
            }
        }

        /**
         * @brief Get the name of the contact sensor.
         * @return The name of the sensor as a string.
         */
        std::string getName() const
        {
            return name_;
        }

        /**
         * @brief Check if the contact sensor is in contact with another object.
         * @return True if the sensor is in contact, false otherwise.
         */
        bool isContact() const
        {
            assert(isContact_);
            return *isContact_;
        }

    private:
        std::string name_;          /**< The name of the contact sensor. */
        const bool *isContact_ = {nullptr};   /**< A pointer to a boolean indicating the contact status of the sensor. */
    };

    /**
     * @brief The ContactSensorInterface class is a hardware interface that manages contact sensor resources.
     *        It inherits from the HardwareResourceManager class and uses the DontClaimResources mechanism.
     */
    class ContactSensorInterface
        : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources>
    {
    };

} // namespace pointfoot_gazebo