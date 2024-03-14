from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('vel', 1.)
sl.declare_arg('type', 'target')


def launch_setup():

    target = sl.arg('type')
    vel = sl.arg('vel') if target == 'target' else 0.

    with sl.group(ns = target):

        sl.robot_state_publisher('integ_description', f'{target}.xacro', xacro_args={'vel': vel})

        sl.spawn_gz_model(target)

        gz_js_topic = GazeboBridge.model_prefix(target) + '/joint_state'
        sl.create_gz_bridge(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)






