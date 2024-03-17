This computed torque control package is nearly finished. Tests are remaining especialy with the purpose of tuning gains.
The robot have to be uploaded in the world in effort mode.
Currently, a sliders:=True mode is accessible when running the launch file intending to mannualy tune the gains.
The package exploits a topic named "desired_joint_states", JointStates type, which holds frequently published target JointState (the frequency is to be every 100ms)
