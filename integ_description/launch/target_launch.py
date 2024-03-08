from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    sl.declare_arg('vel', 1.)

    with sl.group(ns = 'target'):

        sl.robot_state_publisher('integ_description', 'target.xacro', xacro_args=sl.arg_map('vel'))

        sl.spawn_gz_model('target')

        gz_js_topic = GazeboBridge.model_prefix('target') + '/joint_state'
        sl.create_gz_bridge(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

    return sl.launch_description()
