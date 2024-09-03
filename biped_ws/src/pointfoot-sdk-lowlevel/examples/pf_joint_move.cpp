/**
 * @file pf_joint_move.cpp
 * @brief Implementation file for controlling single-joint movement of a robot.
 * @version 1.0
 * @date 2024-3-6
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include "pf_controller_base.h" // Include header file for PFControllerBase class

// Class for controlling single-joint movement inheriting from PFControllerBase
class PFJointMove : public PFControllerBase
{
public:
  /**
   * @brief Initialize the controller.
   */
  void init()
  {
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
  }

  /**
   * @brief Start the control loop for single-joint movement.
   */
  void starting()
  {
    std::cout << "Waiting to receive data...\n";

    while (true)
    {
      if (robotstate_on_)
      {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
        double jointPos = 0; // Variable to store the desired joint position
        double r = 0.0;       // Variable to calculate interpolation ratio

        // If it's the first iteration, initialize the joint position
        if (is_first_enter_)
        {
          joint_init_pos_ = robot_state_.q[joint_id];
          is_first_enter_ = false;
          std::cout << "Received\n";
        }

        // Calculate the interpolation ratio
        r = std::min(std::max(double(running_iter_) / 2000.0, 0.0), 1.0);

        // Calculate the desired joint position using linear interpolation
        jointPos = (1 - r) * (joint_init_pos_ + joint_limit_[joint_id] - joint_offset_[joint_id]) + r * joint_targetPos;

        // Control the joint using a PID controller
        singleJointController(joint_id, joint_kp, joint_kd, jointPos, joint_targetVel, joint_targetTorque);

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
  int joint_id = 0;                 // ID of the controlled joint
  double joint_kp = 60;             // Proportional gain of the PID controller
  double joint_kd = 3;              // Derivative gain of the PID controller
  double joint_targetPos = 0.0;     // Target position for the joint
  double joint_targetVel = 0;       // Target velocity for the joint (not used in this implementation)
  double joint_targetTorque = 0;    // Target torque for the joint (not used in this implementation)

  double joint_init_pos_ = 0;       // Initial position of the joint
  bool is_first_enter_{true};       // Flag to indicate the first iteration
  int running_iter_{1};              // Iteration count
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

  PFJointMove ctrl; // Create an instance of PFJointMove controller
  ctrl.init();      // Initialize the controller
  ctrl.starting();  // Start the control loop

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
