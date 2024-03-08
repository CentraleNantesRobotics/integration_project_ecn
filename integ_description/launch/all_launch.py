from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    for prefix in ('target', 'upload'):
        sl.include('integ_description', f'{prefix}_launch.py')

    return sl.launch_description()
