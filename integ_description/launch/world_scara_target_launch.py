from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('mode', 'velocity')
    sl.declare_arg('sliders', False)  # for manual control
    sl.declare_arg('vel', 1.)
    sl.declare_arg('lambda', 1.) # visual servoing proportional gain

    sl.include('integ_description', 'world_launch.py',
               launch_arguments = {'use_sim_time': True, 'gui': sl.arg('gui')})
    
    sl.include('integ_description', 'upload_launch.py',
               launch_arguments = {'use_sim_time': True, 'mode': sl.arg('mode'), 'sliders': sl.arg('sliders')})

    sl.include('integ_description', 'target_launch.py',
               launch_arguments = {'use_sim_time': True, 'vel': sl.arg('vel')})
    
    # launch FeatureDetector and tracker which are python programs
    sl.node(package='visual_servoing', executable='FeatureDetector.py', output='screen')

    sl.node(package='visual_servoing', executable='tracker.py', output='screen', arguments=[sl.arg('lambda')])
        
    return sl.launch_description()

