class rewards_original:
    reward_type = 'original'  # original, anybipe
    class scales:
        action_rate = -0.01
        ang_vel_xy = -0.05
        base_height = -10.0
        collision = -50.0
        dof_acc = -2.5e-07
        dof_pos_limits = -0.0
        dof_vel = -0.0
        feet_air_time = 60
        feet_contact_forces = -0.01
        feet_stumble = -0.0
        lin_vel_z = -0.5
        no_fly = 1.0
        orientation = -5.0
        stand_still = -10.0
        termination = -0.0
        torque_limits = -0.1
        torques = -2.5e-05
        tracking_ang_vel = 5
        tracking_lin_vel = 10.0
        unbalance_feet_air_time = -300.0
        unbalance_feet_height = -60.0
        feet_distance = -100
        survival = 150

    base_height_target = 0.65
    soft_dof_pos_limit = 0.95  # percentage of urdf limits, values above this limit are penalized
    soft_dof_vel_limit = 0.9
    soft_torque_limit = 0.8
    max_contact_force = 200.  # forces above this value are penalized
    only_positive_rewards = False  # if true negative total rewards are clipped at zero (avoids early termination problems)
    min_feet_distance = 0.1
    min_feet_air_time = 0.25
    max_feet_air_time = 0.65
    tracking_sigma = 0.25  # tracking reward = exp(-error^2/sigma)

class rewards_anybipe:
    reward_type = 'anybipe'
    tracking_sigma = 0.25          # Can be used for command curriculum, if enabled
    tracking_sigma_yaw = 0.25      # Can be used for command curriculum, if enabled
    base_height_target = 0.65
    soft_dof_pos_limit = 0.95  # percentage of urdf limits, values above this limit are penalized
    soft_dof_vel_limit = 0.9
    soft_torque_limit = 0.8
    max_contact_force = 200.
    min_feet_distance = 0.1
    min_feet_air_time = 0.25
    max_feet_air_time = 0.65
    tracking_sigma = 0.25
    class scales:
        tracking_lin_vel = 1.0     # Used for command curriculum, if enabled
        tracking_ang_vel = 0.5     # Used for command curriculum, if enabled
        pass
