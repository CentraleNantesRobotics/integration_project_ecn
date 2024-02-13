from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('mode', 'position')
    sl.declare_arg('jsp', False)

    with sl.group(ns = 'scara'):
        sl.robot_state_publisher('integ_description', 'scara_sw.xacro', xacro_args=sl.arg_map('mode'))

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)

    sl.rviz(sl.find('integ_description', 'config.rviz'))
        
    return sl.launch_description()
