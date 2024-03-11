from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('sliders', True)    # for manual control


def launch_setup():

    sl.node('integ_trajec','integ_trajec_clt')
    sl.node('integ_trajec','integ_trajec_srv')
                        

     # sliders
    if sl.arg('sliders'):
    	sl.node('slider_publisher', arguments = [sl.find('integ_trajec', f'trajec_manual.yaml')])
        

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
