from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    joints = ('right_e0', 'left_e0')
    joints = ("left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2",
    "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2")

    for joint in joints:
        with sl.group(ns = joint):
            sl.include('move_joint', 'slider_launch.py', launch_arguments = [('name', joint)])
    
    return sl.launch_description() 
