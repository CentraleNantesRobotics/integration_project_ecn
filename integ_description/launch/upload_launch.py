from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    sl.include('integ_description', 'state_publisher_launch.py',
               launch_arguments = {'use_sim_time': True})
                    
    # URDF spawner to Gazebo, defaults to relative robot_description topic
    sl.spawn_gz_model('scara')
            
    # ROS-Gz bridges
    bridges = []
    gz_js_topic = GazeboBridge.model_prefix('scara') + '/joint_state'
    bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

    sl.create_gz_bridge(bridges)

    return sl.launch_description()
