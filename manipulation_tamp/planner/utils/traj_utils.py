from drake import lcmt_manipulator_traj, lcmt_multi_wp_manip_query

def lcmt_manipulator_traj_to_dict(msg):
    """ Converts lcmt_manipulator_traj object to python dict

    Args:
        msg: (lcmt_manipulator_traj) decoded message
    Returns:
        d: python dict containing the data
    """

    d = dict()
    d["n_time_steps"] = msg.n_time_steps
    d["dim_torques"] = msg.dim_torques
    d["dim_states"] = msg.dim_states
    d["times_sec"] = msg.times_sec
    d["states"] = msg.states
    d["torques"] = msg.torques
    d["gripper_width"] = msg.gripper_width
    d["gripper_force"] = msg.gripper_force

    d["cost"] = msg.cost

    return d

def dict_to_lcmt_manipulator_traj(d):
    """ Converts python dict to lcmt_manipulator_traj object

    Args:
        d: (dict) python dict containing trajectory data
    Returns:
        msg: (lcmt_manipulator_traj)
    """

    msg = lcmt_manipulator_traj()
    msg.n_time_steps = d["n_time_steps"]
    msg.dim_torques = d["dim_torques"]
    msg.dim_states = d["dim_states"]
    msg.times_sec = d["times_sec"]
    msg.states = d["states"]
    msg.torques = d["torques"]
    msg.gripper_width = d["gripper_width"]
    msg.gripper_force = d["gripper_force"]
    msg.cost = d["cost"]

    return msg

def lcmt_multi_wp_manip_query_to_dict(msg):
    d = dict()
    d["name"] = msg.name

    d["bypass_ik"] = msg.bypass_ik
    d["option"] = msg.option

    d["dim_q"] = msg.dim_q
    d["n_wp"] = msg.n_wp

    d["time_horizon"] = msg.time_horizon
    d["time_step"] = msg.time_step

    d["desired_ee"] = msg.desired_ee
    d["q_goal"] = msg.q_goal

    d["gripper_force"] = msg.gripper_force

    d["prev_gripper_width"] = msg.prev_gripper_width
    d["prev_q"] = msg.prev_q

    d["constrain_orientation"] = msg.constrain_orientation

    try: # for backward compatibility
        d["use_object"] = msg.use_object
        d["q_init_object"] = msg.q_init_object
        d["q_desired_object"] = msg.q_desired_object
    except:
        print("no object pose available")
    
    try:
        d["wait_time"] = msg.wait_time
    except:
        print("No wait_time available")

    return d

def dict_to_lcmt_multi_wp_manip_query(d):
    msg = lcmt_multi_wp_manip_query()
    msg.name = d["name"]

    msg.bypass_ik = d["bypass_ik"]
    msg.option = d["option"]

    msg.dim_q = d["dim_q"]
    msg.n_wp = d["n_wp"]

    msg.time_horizon = d["time_horizon"]
    msg.time_step = d["time_step"]

    msg.desired_ee = d["desired_ee"]
    msg.q_goal = d["q_goal"]

    msg.gripper_force = d["gripper_force"]

    msg.prev_gripper_width = d["prev_gripper_width"]
    msg.prev_q = d["prev_q"]

    msg.constrain_orientation = d["constrain_orientation"]

    if "use_object" in d: # for backward compatibility
        msg.use_object = d["use_object"]
        msg.q_init_object = d["q_init_object"]
        msg.q_desired_object = d["q_desired_object"]
    else:
        msg.use_object = False
        msg.q_init_object = [0 for _ in range(13)]
        msg.q_desired_object = []
        for i in range(msg.n_wp):
            msg.q_desired_object.append([0 for _ in range(13)])
        
    if "wait_time" in d:
        msg.wait_time = d["wait_time"]
    else:
        msg.wait_time = 0
        
    return msg


def get_joint_velocity_from_traj(msg):
    """
    Input: 
        msg: lcmt_manipulator_traj
    output:
        joint_vel: [n_time_steps-1]x[dim_states] matrix for joint velocity
    """
    joint_vel = []
    
    for i in range(msg.n_time_steps-1):
        vel = []
        for j in range(msg.dim_states):
            dq = msg.states[i+1][j] - msg[i][j]
            dt = msg.times_sec[i+1] - msg.times_sec[i]
            vel.append(dq/dt)
        joint_vel.append(vel)
    
    return joint_vel
    

