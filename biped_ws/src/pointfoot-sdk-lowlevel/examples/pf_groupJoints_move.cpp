/**
 * @file pf_groupJoints_move.cpp
 * @brief Implementation file for controlling movement of multiple joints simultaneously.
 * @version 1.0
 * @date 2024-3-6
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include "pf_controller_base.h" // Include header file for PFControllerBase class

// Class for controlling movement of multiple joints simultaneously inheriting from PFControllerBase
class PFGroupJointMove : public PFControllerBase
{
public:
  /**
   * @brief Initialize the controller.
   */
  void init()
  {
    // Set default values for gains, target positions, velocities, and torques
    kp = {60, 60, 60, 60, 60, 60};
    kd = {3, 3, 3, 3, 3, 3};
    targetPos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    targetVel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    targetTorque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<float> offset; // Vector to store joint offsets
    std::vector<float> limit;  // Vector to store joint limits

    // Retrieve joint offsets from the robot
    if (pf_->getJointOffset(offset))
    {
      joint_offset_ << offset[0], offset[1], offset[2],
          offset[3], offset[4], offset[5];
    }
    // Retrieve joint limits from the robot
    if (pf_->getJointLimit(limit))
    {
      joint_limit_ << limit[0], limit[1], limit[2],
          limit[3], limit[4], limit[5];
    }

    robotstate_on_ = false; // Initialize robot state flag
  }

  /**
   * @brief Start the control loop for controlling multiple joints simultaneously.
   */
  void starting()
  {
    std::cout << "Waiting to receive data...\n";

    while (true)
    {
      if (robotstate_on_)
      {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
        std::vector<float> jointPos{6, 0.0}; // Vector to store desired joint positions
        double r = 0.0;                       // Variable to calculate interpolation ratio

        // If it's the first iteration, initialize the joint positions
        if (is_first_enter_)
        {
          init_pos_ = robot_state_.q;
          is_first_enter_ = false;
          std::cout << "Received\n";
        }

        // Calculate the interpolation ratio
        r = std::min(std::max(double(running_iter_) / 2000.0, 0.0), 1.0);

        // Calculate the desired joint positions using linear interpolation
        for (size_t i = 0; i < getNumofJoint(); ++i)
        {
          jointPos[i] = (1 - r) * (init_pos_[i] + joint_limit_[i] - joint_offset_[i]) + r * targetPos[i];
        }

        // Control the joints using PID controllers
        groupJointController(kp, kd, jointPos, targetVel, targetTorque);

        std::this_thread::sleep_until(time_point); // Sleep until the next iteration

        if (!is_first_enter_)
        {
          running_iter_++; // Increment the iteration count
        }
        robotstate_on_ = false; // Reset the flag for receiving robot state data
      }
      else
      {
        usleep(1); // Sleep for a short duration if robot state data is not received
      }
    }
  }

private:
  std::vector<float> kp{6, 0.0}, kd{6, 0.0}, targetPos{6, 0.0}, targetVel{6, 0.0}, targetTorque{6, 0.0}; // Gains and targets
  std::vector<float> init_pos_{6, 0.0};                                                                   // Initial joint positions
  bool is_first_enter_{true};                                                                              // Flag for first iteration
  int running_iter_{1};                                                                                    // Iteration count
};

/**
 * @brief Main function.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer indicating the exit status.
 */
int main(int argc, char *argv[])
{
  limxsdk::PointFoot *pf = limxsdk::PointFoot::getInstance(); // Obtain instance of PointFoot class

  std::string robot_ip = "127.0.0.1"; // Default robot IP address
  if (argc > 1)
  {
    robot_ip = argv[1]; // Use command-line argument as robot IP address if provided
  }

  // Initialize the robot
  if (!pf->init(robot_ip))
  {
    exit(1); // Exit program if initialization fails
  }

  PFGroupJointMove ctrl; // Create an instance of PFGroupJointMove controller
  ctrl.init();           // Initialize the controller
  ctrl.starting();       // Start the control loop

  // Infinite loop to keep the program running
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 100 milliseconds
  }

#ifdef WIN32
  timeEndPeriod(1);
#endif

  return 0; // Return 0 to indicate successful execution
}
