from legged_gym.envs.base.base_config import BaseConfig

class PointFootRoughCfg(BaseConfig):
    class env:
        num_envs = 4096
        num_propriceptive_obs = 27
        num_privileged_obs = 148  # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise
        num_actions = 6
        env_spacing = 3.  # not used with heightfields/trimeshes
        send_timeouts = True  # send time out information to the algorithm
        episode_length_s = 20  # episode length in seconds

    class terrain:
        mesh_type = 'trimesh'  # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1  # [m]
        vertical_scale = 0.005  # [m]
        border_size = 25  # [m]
        curriculum = True
        static_friction = 0.4
        dynamic_friction = 0.6
        restitution = 0.8
        # rough terrain only:
        measure_heights_actor = False
        measure_heights_critic = True
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4,
                             0.5]  # 1mx1m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False  # select a unique terrain type and pass all arguments
        terrain_kwargs = None  # Dict of arguments for selected terrain
        max_init_terrain_level = 5  # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 10  # number of terrain rows (levels)
        num_cols = 20  # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.1, 0.1, 0.35, 0.25, 0.2]
        # trimesh only:
        slope_treshold = 0.75  # slopes above this threshold will be corrected to vertical surfaces

    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 4  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 50.  # time before command are changed[s]
        heading_command = True  # if true: compute ang vel command from heading error

        class ranges:
            lin_vel_x = [0.0, 2.0]  # min max [m/s]
            lin_vel_y = [0.0, 0.1]  # min max [m/s]
            ang_vel_yaw = [-1, 1]  # min max [rad/s]
            heading = [-3.14, 3.14]

    class init_state:
        pos = [0.0, 0.0, 0.62]  # x,y,z [m]
        rot = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w [quat]
        lin_vel = [0.0, 0.0, 0.0]  # x,y,z [m/s]
        ang_vel = [0.0, 0.0, 0.0]  # x,y,z [rad/s]
        default_joint_angles = {  # target angles when action = 0.0
            "abad_L_Joint": 0.0,
            "hip_L_Joint": 0.0,
            "knee_L_Joint": 0.0,
            "foot_L_Joint": 0.0,
            "abad_R_Joint": 0.0,
            "hip_R_Joint": 0.0,
            "knee_R_Joint": 0.0,
            "foot_R_Joint": 0.0,
        }

    class control:
        control_type = 'P'  # P: position, V: velocity, T: torques
        # PD Drive parameters:
        stiffness = {
            "abad_L_Joint": 40,
            "hip_L_Joint": 40,
            "knee_L_Joint": 40,
            "foot_L_Joint": 0.0,
            "abad_R_Joint": 40,
            "hip_R_Joint": 40,
            "knee_R_Joint": 40,
            "foot_R_Joint": 0.0,
        }  # [N*m/rad]
        damping = {
            "abad_L_Joint": 1.5,
            "hip_L_Joint": 1.5,
            "knee_L_Joint": 1.5,
            "foot_L_Joint": 0.0,
            "abad_R_Joint": 1.5,
            "hip_R_Joint": 1.5,
            "knee_R_Joint": 1.5,
            "foot_R_Joint": 0.0,
        }  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset:
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/PF_P441A/urdf/PF_P441A.urdf'
        name = "PF_P441A"
        foot_name = 'foot'
        terminate_after_contacts_on = ["abad", "base"]
        penalize_contacts_on = ["base", "abad", "hip", "knee"]
        disable_gravity = False
        collapse_fixed_joints = True  # merge bodies connected by fixed joints. Specific fixed joints can be kept by adding " <... dont_collapse="true">
        fix_base_link = False  # fixe the base of the robot
        default_dof_drive_mode = 3  # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
        replace_cylinder_with_capsule = True  # replace collision cylinders with capsules, leads to faster/more stable simulation
        flip_visual_attachments = False  # Some .obj meshes must be flipped from y-up to z-up

        density = 0.001
        angular_damping = 0.
        linear_damping = 0.
        max_angular_velocity = 1000.
        max_linear_velocity = 1000.
        armature = 0.
        thickness = 0.01

    class domain_rand:
        rand_interval_s = 10
        randomize_friction = True
        friction_range = [0.0, 1.6]
        randomize_base_mass = True
        added_mass_range = [-1., 2.]
        randomize_base_com = True
        rand_com_vec = [0.03, 0.02, 0.03]
        randomize_restitution = True
        restitution_range = [0.0, 0.8]
        restitution = 0.4
        randomize_motor_strength = True
        motor_strength_range = [0.9, 1.1]
        push_robots = True
        push_interval_s = 7
        max_push_vel_xy = 1.
        randomize_Kp_factor = False
        Kp_factor_range = None
        randomize_Kd_factor = False
        Kd_factor_range = None

        randomize_gravity = False
        gravity_rand_interval_s = 10
        randomize_rolling_friction = False
        randomize_torsion_friction = False
        randomize_dof_stiffness = False
        randomize_dof_damping = False
        randomize_dof_friction = False
        randomize_dof_armature = False
        randomize_com_displacement = False

    class domain_rand_anybipe:
        rand_interval_s = 10
        friction_range = None
        rolling_friction_range = None
        torsion_friction_range = None
        restitution_range = None
        added_mass_range = None
        com_displacement_range = None
        motor_strength_range = None
        Kp_factor_range = None
        Kd_factor_range = None
        dof_stiffness_range = None
        dof_damping_range = None
        dof_friction_range = None
        dof_armature_range = None
        push_vel_xy_range = None  # Hack to format this as range
        max_push_vel_xy = None
        push_interval_s = 7
        gravity_range = None
        rand_com_vec = None
        gravity_rand_interval_s = 10

# INSERT ANYBIPE DR HERE (NOT IMPLEMENTED YET)

        randomize_friction = False if friction_range is None else True
        randomize_rolling_friction = False if rolling_friction_range is None else True
        randomize_torsion_friction = False if torsion_friction_range is None else True
        randomize_restitution = False if restitution_range is None else True
        randomize_base_mass = False if added_mass_range is None else True
        randomize_com_displacement = False if com_displacement_range is None else True
        randomize_motor_strength = False if motor_strength_range is None else True
        randomize_Kp_factor = False if Kp_factor_range is None else True
        randomize_Kd_factor = False if Kd_factor_range is None else True
        randomize_dof_stiffness = False if dof_stiffness_range is None else True
        randomize_dof_damping = False if dof_damping_range is None else True
        randomize_dof_friction = False if dof_friction_range is None else True
        randomize_dof_armature = False if dof_armature_range is None else True
        max_push_vel_xy = None if push_vel_xy_range is None else push_vel_xy_range[1]
        push_robots = False if max_push_vel_xy is None else True
        randomize_gravity = False if gravity_range is None else True
        randomize_base_com = False if rand_com_vec is None else True


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

        base_height_target = 0.62
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
        reward_type = 'anybipe'  # original, anybipe
        tracking_sigma = 0.25          # Can be used for command curriculum, if enabled
        orientation_sigma = 0.25       # Can be used for command curriculum, if enabled
        tracking_sigma_yaw = 0.25      # Can be used for command curriculum, if enabled
        base_height_target = 0.62
        soft_dof_pos_limit = 0.95  # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.8
        class scales:
            tracking_lin_vel = 1.0     # Used for command curriculum, if enabled
            tracking_ang_vel = 0.5     # Used for command curriculum, if enabled
            pass

    class normalization:
        class obs_scales:
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.05
            height_measurements = 5.0

        clip_observations = 100.
        clip_actions = 100.

    class noise:
        add_noise = True
        noise_level = 1.0  # scales other values

        class noise_scales:
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05
            height_measurements = 0.1

    # viewer camera:
    class viewer:
        ref_env = 0
        pos = [10, 0, 6]  # [m]
        lookat = [11., 5, 3.]  # [m]

    class sim:
        dt = 0.005
        substeps = 1
        gravity = [0., 0., -9.81]  # [m/s^2]
        up_axis = 1  # 0 is y, 1 is z

        class physx:
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 4
            num_velocity_iterations = 0
            contact_offset = 0.01  # [m]
            rest_offset = 0.0  # [m]
            bounce_threshold_velocity = 0.5  # 0.5 [m/s]
            max_depenetration_velocity = 1.0
            max_gpu_contact_pairs = 2 ** 23  # 2**24 -> needed for 8000 envs and more
            default_buffer_size_multiplier = 5
            contact_collection = 2  # 0: never, 1: last sub-step, 2: all sub-steps (default=2)


class PointFootRoughCfgPPO(BaseConfig):
    seed = 1
    runner_class_name = 'OnPolicyRunner'

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu'  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        # only for 'ActorCriticRecurrent':
        # rnn_type = 'lstm'
        # rnn_hidden_size = 512
        # rnn_num_layers = 1

    class algorithm:
        # training params
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4  # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 1.e-3  # 5.e-4
        schedule = 'adaptive'  # could be adaptive, fixed
        gamma = 0.99
        lam = 0.95
        desired_kl = 0.01
        max_grad_norm = 1.
        # The following parameters are used to load teacher models
        # If your teacher model is realized by MPC, onnx etc. (not pytorch), you should consider to implement a cpu dispatcher to accelerate the inference (or cuda dispatcher)
        use_teacher_model = False
        teacher_function_path = None
        teacher_coef = 0.0


    class runner:
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 24  # per iteration
        max_iterations = 100000  # number of policy updates

        # logging
        save_interval = 500  # check for potential saves every this many iterations
        experiment_name = 'pointfoot_rough'
        run_name = ''
        # load and resume
        resume = False
        load_run = -1  # -1 = last run
        checkpoint = -1  # -1 = last saved model
        resume_path = None  # updated from load_run and chkpt
