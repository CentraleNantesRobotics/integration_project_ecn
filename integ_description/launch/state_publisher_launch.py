from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('jsp', False)

    sl.robot_state_publisher('integ_description', 'scara_sw.urdf')

    sl.rviz(sl.find('integ_description', 'config.rviz'))

    with sl.group(if_arg='jsp'):
        sl.joint_state_publisher(True)
        
    return sl.launch_description()
