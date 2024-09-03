/**
 * @file PointFootHWSim.cpp
 *
 * @brief This file defines the PointFootHWSim class, which is responsible for simulating a legged robot.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "pointfoot_gazebo/PointFootHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

static limxsdk::PointFootSim *pf;

namespace pointfoot_gazebo
{
    bool PointFootHWSim::initSim(const std::string &robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                          const urdf::Model *urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        // Initialize the PointFootSim instance and connect to the robot at IP address 127.0.0.1
        pf = limxsdk::PointFootSim::getInstance();
        pf->init("127.0.0.1");

        // Write the number of motors in the robotCmdBuffer_
        robotCmdBuffer_.writeFromNonRT(limxsdk::RobotCmd(pf->getMotorNumber()));

        // Subscribe to robot command messages and write them to robotCmdBuffer_
        pf->subscribeRobotCmdForSim([this](const limxsdk::RobotCmdConstPtr& msg) 
        {
            robotCmdBuffer_.writeFromNonRT(*msg);
        });

        // Call the initSim function of the DefaultRobotHWSim class
        bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

        // Register the hybridJointInterface_
        registerInterface(&hybridJointInterface_);

        // Get the names of the joints from ej_interface_
        std::vector<std::string> names = ej_interface_.getNames();

        // Iterate through each joint name
        for (const auto &name : names)
        {
            // Push a new HybridJointData object with the joint handle to hybridJointDatas_
            hybridJointDatas_.push_back(HybridJointData{.joint_ = ej_interface_.getHandle(name)});
            HybridJointData &back = hybridJointDatas_.back();
            
            // Register the HybridJointHandle with the hybridJointInterface_
            hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
            
            // Insert a new pair into cmdBuffer_ with the joint name as key and an empty deque as value
            cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
        }

        // Register the imuSensorInterface_
        registerInterface(&imuSensorInterface_);

        // Get the IMU configurations from the model_nh parameter "gazebo/imus"
        XmlRpc::XmlRpcValue xmlRpcValue;
        if (!model_nh.getParam("gazebo/imus", xmlRpcValue))
        {
            ROS_WARN("No imu specified");
        }
        else
        {
            parseImu(xmlRpcValue, parent_model);
        }

        // Get the delay value from the model_nh parameter "gazebo/delay"
        if (!model_nh.getParam("gazebo/delay", delay_))
        {
            delay_ = 0.;
        }

        // Get the contact configurations from the model_nh parameter "gazebo/contacts"
        if (!model_nh.getParam("gazebo/contacts", xmlRpcValue))
        {
            ROS_WARN("No contacts specified");
        }
        else
        {
            parseContacts(xmlRpcValue);
        }

        // Set the contactManager_ to the ContactManager of the parent model's world physics
        contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();

        // Set the contactManager_ to never drop contacts (if false, we need to select view->contacts in gazebo GUI to avoid returning nothing when calling ContactManager::GetContacts())
        contactManager_->SetNeverDropContacts(true);

        return ret;
    }

    void PointFootHWSim::readSim(ros::Time time, ros::Duration period)
    {
        gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
        // Imu Sensor
        std::random_device seed;                            // Generate a random number seed from hardware
        std::ranlux48 engine(seed());                       // Use the seed to create a random number engine
        std::normal_distribution<double> distrib(0.0, 1.0); // Define a random number distribution object 'distrib' with mean 0.0 and standard deviation 1.0 of a normal distribution

        // Update IMU sensor data
        for (auto &imu : imuDatas_)
        {
            // Get pose and orientation values from the world frame
            ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
            imu.ori_[0] = pose.Rot().X();
            imu.ori_[1] = pose.Rot().Y();
            imu.ori_[2] = pose.Rot().Z();
            imu.ori_[3] = pose.Rot().W();

            // Get angular velocity values in the body frame
            ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
            imu.angularVel_[0] = rate.X(); //+ distrib(engine) * 1e-1;
            imu.angularVel_[1] = rate.Y(); //+ distrib(engine) * 1e-1;
            imu.angularVel_[2] = rate.Z(); //+ distrib(engine) * 1e-1;

            // Calculate linear acceleration by subtracting gravity in the world frame
            ignition::math::Vector3d gravity = {0., 0., -9.81};
            ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
            imu.linearAcc_[0] = accel.X(); //+ distrib(engine) * 1e-1;
            imu.linearAcc_[1] = accel.Y(); //+ distrib(engine) * 1e-1;
            imu.linearAcc_[2] = accel.Z(); //+ distrib(engine) * 1e-1;
        }

        // Update contact sensor data
        for (auto &state : name2contact_)
        {
            state.second = false;
        }
        for (const auto &contact : contactManager_->GetContacts())
        {
            if (static_cast<uint32_t>(contact->time.sec) != time.sec || static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec)
            {
                continue;
            }
            std::string linkName = contact->collision1->GetLink()->GetName();
            if (name2contact_.find(linkName) != name2contact_.end())
            {
                name2contact_[linkName] = true;
            }
            linkName = contact->collision2->GetLink()->GetName();
            if (name2contact_.find(linkName) != name2contact_.end())
            {
                name2contact_[linkName] = true;
            }
        }

        // Publish robot state for simulation
        if (joint_names_.size() == pf->getMotorNumber()) 
        {
            limxsdk::RobotState state;

            state.stamp = ros::Time::now().toNSec();
            state.q.resize(joint_names_.size());
            state.dq.resize(joint_names_.size());
            state.tau.resize(joint_names_.size());

            // Fill in joint position, velocity, and effort values
            for (int i = 0; i < joint_names_.size(); i++) 
            {
                int index = parseJointIndex(joint_names_[i]);
                if (index != -1) 
                {
                    ros::Time now = ros::Time::now();
                    state.stamp = now.toNSec();
                    state.q[index] = joint_position_[i];
                    state.dq[index] = joint_velocity_[i];
                    state.tau[index] = joint_effort_[i];
                }
            }

            pf->publishRobotStateForSim(state);
            
            // Fill in IMU sensor data
            limxsdk::ImuData imu_data;
            imu_data.stamp = ros::Time::now().toNSec();
            if (imuDatas_.size() > 0) 
            {
                auto &imu = imuDatas_.front();
                imu_data.acc[0] = imu.linearAcc_[0]; // X
                imu_data.acc[1] = imu.linearAcc_[1]; // Y
                imu_data.acc[2] = imu.linearAcc_[2]; // Z
                imu_data.quat[1] = imu.ori_[0]; // X
                imu_data.quat[2] = imu.ori_[1]; // Y
                imu_data.quat[3] = imu.ori_[2]; // Z
                imu_data.quat[0] = imu.ori_[3]; // W
                imu_data.gyro[0] = imu.angularVel_[0]; // X
                imu_data.gyro[1] = imu.angularVel_[1]; // Y
                imu_data.gyro[2] = imu.angularVel_[2]; // Z
            }
            pf->publishImuDataForSim(imu_data);
        }
        else
        {
            ROS_ERROR("joint_names size(%ld) != RobotMotorNumber(%u)", joint_names_.size(), pf->getMotorNumber());
        }

        // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
        for (auto &cmd : joint_effort_command_)
        {
            cmd = 0;
        }
        for (auto &cmd : joint_velocity_command_)
        {
            cmd = 0;
        }
    }

    void PointFootHWSim::writeSim(ros::Time time, ros::Duration period)
    {
        auto robot_cmd = *robotCmdBuffer_.readFromRT();

        // Update joint data with commands from robot command buffer
        for (auto & joint : hybridJointDatas_) 
        {
            int index = parseJointIndex(joint.joint_.getName());
            if (index != -1) 
            {
                // Set desired position, velocity, feedforward term, kp, and kd for the joint
                joint.posDes_ = robot_cmd.q[index];
                joint.velDes_ = robot_cmd.dq[index];
                joint.ff_ = robot_cmd.tau[index];
                joint.kp_ = robot_cmd.Kp[index];
                joint.kd_ = robot_cmd.Kd[index];
            }
        }

        // Update joint commands in the cmdBuffer based on simulation time
        for (auto joint : hybridJointDatas_)
        {
            auto &buffer = cmdBuffer_.find(joint.joint_.getName())->second;

            if (time == ros::Time(period.toSec()))
            { // Simulation reset
                buffer.clear();
            }

            // Remove old commands from the buffer that are older than delay_
            while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time)
            {
                buffer.pop_back();
            }

            // Push new command to the front of the buffer
            buffer.push_front(HybridJointCommand{
                .stamp_ = time, .posDes_ = joint.posDes_, .velDes_ = joint.velDes_, .kp_ = joint.kp_, .kd_ = joint.kd_, .ff_ = joint.ff_});

            const auto &cmd = buffer.back();
            // Set the joint command by calculating the control output using pid controller equation
            joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
                                    cmd.ff_);
        }

        // Call the writeSim() function of the parent class to perform additional simulation updates
        DefaultRobotHWSim::writeSim(time, period);
    }

    void PointFootHWSim::parseImu(XmlRpc::XmlRpcValue &imuDatas, const gazebo::physics::ModelPtr &parentModel)
    {
        // Check if imuDatas is of type XmlRpc::XmlRpcValue::TypeStruct
        ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        // Iterate over each element in imuDatas
        for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it)
        {
            // Check if the current imuData has "frame_id" member
            if (!it->second.hasMember("frame_id"))
            {
                ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
                continue;
            }
            // Check if the current imuData has "orientation_covariance_diagonal" member
            else if (!it->second.hasMember("orientation_covariance_diagonal"))
            {
                ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
                continue;
            }
            // Check if the current imuData has "angular_velocity_covariance" member
            else if (!it->second.hasMember("angular_velocity_covariance"))
            {
                ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
                continue;
            }
            // Check if the current imuData has "linear_acceleration_covariance" member
            else if (!it->second.hasMember("linear_acceleration_covariance"))
            {
                ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
                continue;
            }

            // Get the orientation covariance values from imuDatas
            XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
            ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(oriCov.size() == 3);
            for (int i = 0; i < oriCov.size(); ++i)
            {
                ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            }

            // Get the angular velocity covariance values from imuDatas
            XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
            ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(angularCov.size() == 3);
            for (int i = 0; i < angularCov.size(); ++i)
            {
                ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            }

            // Get the linear acceleration covariance values from imuDatas
            XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
            ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(linearCov.size() == 3);
            for (int i = 0; i < linearCov.size(); ++i)
            {
                ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            }

            // Get the frameId of the current imuData
            std::string frameId = imuDatas[it->first]["frame_id"];

            // Get the link pointer associated with the frameId from parentModel
            gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
            ROS_ASSERT(linkPtr != nullptr);

            // Create an ImuData object with the obtained data and push it to imuDatas_ vector
            imuDatas_.push_back((ImuData{
                .linkPtr_ = linkPtr,
                .ori_ = {0., 0., 0., 0.},
                .oriCov_ = {static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0., static_cast<double>(oriCov[2])},
                .angularVel_ = {0., 0., 0.},
                .angularVelCov_ = {static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]), 0., 0., 0.,
                                   static_cast<double>(angularCov[2])},
                .linearAcc_ = {0., 0., 0.},
                .linearAccCov_ = {static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]), 0., 0., 0.,
                                  static_cast<double>(linearCov[2])}}));
            ImuData &imuData = imuDatas_.back();
            imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_,
                                                                                   imuData.angularVel_, imuData.angularVelCov_, imuData.linearAcc_,
                                                                                   imuData.linearAccCov_));
        }
    }

    void PointFootHWSim::parseContacts(XmlRpc::XmlRpcValue &contactNames)
    {
        // Verify that contactNames is of type Array
        ROS_ASSERT(contactNames.getType() == XmlRpc::XmlRpcValue::TypeArray);

        // Iterate through the contact names array
        for (int i = 0; i < contactNames.size(); ++i)
        { // NOLINT(modernize-loop-convert)
            // Get the name at index i and store it in a string variable
            std::string name = contactNames[i];

            // Insert the contact name and corresponding boolean value into the name2contact_ map
            name2contact_.insert(std::make_pair(name, false));

            // Register a contact sensor handle with the given name and boolean reference in contactSensorInterface_
            contactSensorInterface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
        }

        // Register the contactSensorInterface_ with the hardware interface
        registerInterface(&contactSensorInterface_);
    }

    int PointFootHWSim::parseJointIndex(const std::string& jointName) {
        int leg_index = 0;
        int joint_index = 0;

       
        if (jointName.find("L") != std::string::npos)
        {
            leg_index = 0;
        }
        else if (jointName.find("R") != std::string::npos)
        {
            leg_index = 1;
        }
        else
        {
            return -1;
        }

        if (jointName.find("abad") != std::string::npos)
        {
            joint_index = 0;
        }
        else if (jointName.find("hip") != std::string::npos)
        {
            joint_index = 1;
        }
        else if (jointName.find("knee") != std::string::npos)
        {
            joint_index = 2;
        }
        else
        {
            return -1;
        }

        return (leg_index * 3 + joint_index);
    }

} // namespace pointfoot_gazebo

// Export the PointFootHWSim class as a plugin for gazebo_ros_control::RobotHWSim
PLUGINLIB_EXPORT_CLASS(pointfoot_gazebo::PointFootHWSim, gazebo_ros_control::RobotHWSim)

// Register the GazeboRosControlPlugin as a model plugin
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)
