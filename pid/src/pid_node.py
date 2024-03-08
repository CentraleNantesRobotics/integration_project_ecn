#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pid.msg import PIDcommand


class PID(Node):
	def __init__(self):
		self.declare_parameter('P',rclpy.Parameter.Type.FLOAT)
		self.declare_parameter('I',rclpy.Parameter.Type.FLOAT)
		self.declare_parameter('D',rclpy.Parameter.Type.FLOAT)
		self.declare_parameter('q1_windup',rclpy.Parameter.Type.FLOAT)
		self.declare_parameter('q2_windup',rclpy.Parameter.Type.FLOAT)
		self.P = self.get_parameters("PID/P")
		self.I = self.get_parameters("PID/I")
		self.D = self.get_parameters("PID/D")
		self.q1_windup = self.get_parameters("PID/q1_windup")
		self.q2_windup = self.get_parameters("PID/q2_windup")
		self.oldtime = 0
		self.dt = 0

        #a joint state subscriber
		self.joint_subscriber = self.create_subscription(JointState, '/scara/joint_states', self.joint_states_callback, 10)
        # publisher of the joint1 position command
		self.joint1_publisher_ = self.create_publisher(PIDcommand, '/scara/pid_command_torques', 10)

        #subscriber of the trajectory message in joint state #TODO: effector version
		self.trajectory_subscriber = self.create_subscription(JointState, '/trajectory', self.trajectory_callback, 10)

		self.torques_command = PIDcommand()

	def trajectory_subscriber(self,msg):
		self.q1_des = msg.position[0]
		self.q2_des = msg.position[1]
		if len(msg.velocity)==0 :
			self.w1_des = 0
			self.w2_des = 0
		else:
			self.w1_des = msg.velocity[0]
			self.w2_des = msg.velocity[1]

	def joint_states_callback(self, msg):
		self.q1 = msg.position[0]
		self.q2 = msg.position[1]
		self.w1 = msg.velocity[0]
		self.w2 = msg.velocity[1]
		if old_time != 0:
			self.dt = (msg.header.stamp.sec + msg.header.stamp.nsec*1e-9) - oldtime
		self.oldtime = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
		self.torques_command.header = msg.header
		self.PID_fun()

	def PID_fun(self):
        #compute integral
		int_q1 = self.q1 * self.dt #TODO
		int_q2 = self.q2 * self.dt #TODO

        #anti wind-up
		if q1_int > windup_q1:
			q1_int = self.windup_q1
		if q1_int < -windup_q1:
			q1_int = -self.windup_q1
		if q2_int > windup_q2:
			q2_int = self.windup_q2
		if q2_int < -windup_q2:
			q2_int = -self.windup_q2

        #PID id dt!=0
		err1 = self.q1_des - self.q1
		err1_w = self.w1_des - self.w1
		err1_int = 0 - self.q1_int #CHECK
		err2 = self.q2_des - self.q2
		err2_w = self.w2_des - self.w2
		err2_int = 0 - self.q2_int #CHECK

        #PID
		com_q1 = Kp*self.err1 + Kd * self.err_w1 + Ki * self.err1_int
		com_q2 = Kp*self.err2 + Kd * self.err_w2 + Ki * self.err2_int

        #publish command
		torques_command.torques=[com_q1, com_q2]
		self.joint1_publisher_.publish(torques_command)

		q1_prev = self.q1
		q2_prev = self.q2


def main(args=None):
    rclpy.init(args=args)
    # Add your main code here
    PID_object = PID()
    rclpy.spin(PID_object)
    PID_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
