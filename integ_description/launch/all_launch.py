from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('mode', 'velocity')
    sl.declare_arg('sliders', False)  # for manual control
    sl.declare_arg('vel', 1.)
    sl.declare_arg('lambda', 1.)  # visual servoing proportional gain
    sl.declare_arg('use_vs', False)
    sl.declare_arg('debug', False)

    sl.include('integ_description', 'upload_launch.py',
               launch_arguments={'use_sim_time': True, 'mode': sl.arg('mode'), 'sliders': sl.arg('sliders')})

    sl.include('integ_description', 'target_launch.py',
               launch_arguments={'use_sim_time': True, 'vel': sl.arg('vel')})

    with sl.group(if_arg = 'use_vs') :
        with sl.group(if_arg = 'debug'):
            sl.node(package='visual_servoing', executable='FeatureDetector', output='screen')
            sl.node(package='visual_servoing', executable='tracker', output='screen', parameters=[{'lambda': sl.arg('lambda')}])
        with sl.group(unless_arg = 'debug'):
            with sl.group(unless_arg='debug'):
                sl.node(package='visual_servoing', executable='FeatureDetector', output='screen')
                sl.node(package='visual_servoing', executable='tracker', output='screen', parameters=[{'lambda': sl.arg('lambda')}])
        
    return sl.launch_description()
