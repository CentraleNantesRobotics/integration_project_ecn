from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('mode', 'position')
sl.declare_arg('sliders', False)    # for manual control


def launch_setup():

    mode = sl.arg('mode')

    sl.include('integ_description', 'state_publisher_launch.py',
               launch_arguments = {'use_sim_time': True, 'mode': mode})
                        
    # URDF spawner to Gazebo, defaults to relative robot_description topic

    with sl.group(ns = 'scara'):

        sl.spawn_gz_model('scara')

        # sliders
        if sl.arg('sliders'):
            sl.node('slider_publisher', arguments = [sl.find('integ_description', f'{mode}_manual.yaml')])

        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix('scara') + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

        #add a camera bridge
        bridges.append(GazeboBridge('scara/image', '/scara/image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

        for joint in ('joint_1', 'joint_2'):

            if mode == 'position':
                bridges.append(GazeboBridge(f'/model/scara/joint/{joint}/0/cmd_pos',
                                            f'{joint}_cmd_pos',
                                            'std_msgs/Float64',
                                            GazeboBridge.ros2gz))
            elif mode == 'velocity':
                bridges.append(GazeboBridge(f'{joint}_cmd_vel',
                                            f'{joint}_cmd_vel',
                                            'std_msgs/Float64',
                                            GazeboBridge.ros2gz))
            elif mode == 'effort':
                bridges.append(GazeboBridge(f'/model/scara/joint/{joint}/cmd_force',
                                            f'{joint}_cmd_effort',
                                            'std_msgs/Float64',
                                            GazeboBridge.ros2gz))

        sl.create_gz_bridge(bridges)
        

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)