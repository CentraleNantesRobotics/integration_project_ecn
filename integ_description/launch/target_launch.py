from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    sl.declare_arg('vel', 1.)

    with sl.group(ns = 'target'):

        sl.robot_state_publisher('integ_description', 'target.xacro', xacro_args=sl.arg_map('vel'))

        sl.spawn_gz_model('target')

        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix('target') + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

        sl.create_gz_bridge(bridges)

    return sl.launch_description()
