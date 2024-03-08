from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    sl.declare_arg('vel', 1.)

    with sl.group(ns = 'target'):

        sl.robot_state_publisher('integ_description', 'target.xacro', xacro_args=sl.arg_map('vel'))

        sl.spawn_gz_model('target')

    return sl.launch_description()
