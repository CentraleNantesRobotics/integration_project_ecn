from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('sliders', False)    # for manual control


def launch_setup():

    mode = sl.arg('mode')


    sl.node('integ_command','integ_CTC.cpp')
                        

        # sliders
        if sl.arg('sliders'):
            sl.node('slider_publisher', arguments = [sl.find('integ_command', f'CTC_manual.yaml')])
        

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
