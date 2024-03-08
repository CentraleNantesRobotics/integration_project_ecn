from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('gui', default_value=True)

    gz_args = '-r ' + sl.find('integ_description', 'world.sdf')

    with sl.group(if_arg='gui'):
        sl.gz_launch(gz_args)
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch('-s ' + gz_args)
        
    sl.create_gz_clock_bridge()
        
    return sl.launch_description()