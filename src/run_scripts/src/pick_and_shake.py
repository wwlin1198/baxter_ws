#! /usr/bin/env python3
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, PoseArray
from positionControl import *
import time
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np



class Pick_and_shake:

	def object_poses_callback(self, data):
		if len(data.poses) > 0:
			self.object_poses = data.poses 

	# def joy_callback(self,data):
	def get_object_grasp_pose(self, inpose):
		pose = PoseStamped()
		pose.pose.position.x = inpose.pose.position.x - 0.2533
		pose.pose.position.y = inpose.pose.position.y + 0.03866
		pose.pose.position.z = inpose.pose.position.z - 0.49375

		pose.pose.orientation = inpose.pose.orientation
		# pose.pose.orientation.x = 0.7101583320880945
		# pose.pose.orientation.y = -0.7034897541238019
		# pose.pose.orientation.z = 0.001383346838212429
		# pose.pose.orientation.w = -0.027845925379017162

		return pose


	def transform_object_pose_to_robot_rf(self):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_rgb_optical_frame',rospy.Time(0),
			rospy.Duration(1.0))
		print transform 
		print "----------------" 
		rot = transform.transform.rotation
		print tf.transformations.euler_from_quaternion((rot.x,rot.y,rot.z,rot.w))

		# trans_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)

		# return trans_pose


	def grasp_object(self, arm):
		if len(self.object_poses) > 0:
			ind = np.random.randint(low=0, high=len(self.object_poses))
			to_pick = self.object_poses[ind]
			p_stamp = PoseStamped()
			p_stamp.pose = to_pick
			pick_pose = self.transform_object_pose_to_robot_rf(p_stamp)

			
			

			# print(pick_pose)
			pose = self.get_object_grasp_pose(pick_pose)

			if arm == 'left':
				left_arm_follow_trajectory('waypoints/pre_grasp.wp', self.left_arm, self.pause_event)
				#gripper orientation_
				current_pose = self.left_arm.endpoint_pose()
				pose.pose.orientation.x = current_pose['orientation'][0]
				pose.pose.orientation.y = current_pose['orientation'][1]
				pose.pose.orientation.z = current_pose['orientation'][2]
				pose.pose.orientation.w = current_pose['orientation'][3]
				time.sleep(1)
				move_to_goal_pose(self.left_arm, pose, self.pause_event)
				time.sleep(2)
				self.lGripper.close()
				time.sleep(1)

				left_arm_follow_trajectory('waypoints/left_raise.wp', self.left_arm, self.pause_event)


			elif arm == 'right':
				move_to_goal_pose(self.right_arm, pose, self.pause_event)
				time.sleep(2)
				self.rGripper.close()



	def move_to_init(self):
		both_arms_follow_trajectory('waypoints/left_move_to_init.wp', 'waypoints/right_move_to_init.wp', 
			self.left_arm, self.right_arm, self.pause_event)



	def move_to_sleep(self):
		both_arms_follow_trajectory('waypoints/left_move_to_sleep.wp', 'waypoints/right_move_to_sleep.wp', 
			self.left_arm, self.right_arm, self.pause_event)


	def perform_grasp(self, arm):
		self.grasp_object(arm)
		# self.shake(arm)
		# self.place_back(arm)



	def __init__(self):
		# print('starting ros')
		rospy.init_node('pick_n_shake', disable_signals=True)
		self.place = 0
		self.baxter_enabler = baxter.RobotEnable(versioned=True)
		self.baxter_enabler.enable()

		self.left_arm = baxter.Limb('left')
		self.right_arm = baxter.Limb('right')
		self.lGripper = baxter.Gripper('left')
		self.rGripper = baxter.Gripper('right')
		time.sleep(3)
		self.transform_object_pose_to_robot_rf()
		'''
		#calibrating gripper
		if not self.lGripper.calibrate():
		    print("left gripper did not calibrate")
		    sys.exit()
		if not self.rGripper.calibrate():
		    print("right gripper did not calibrate")
		    sys.exit()
		
		self.lGripper.set_holding_force(50)
		self.lGripper.set_moving_force(30)

		self.rGripper.set_holding_force(10)
		self.rGripper.set_moving_force(30)

		rospy.Subscriber('/object_poses', PoseArray, self.object_poses_callback)

		self.head = baxter.Head()
		#self.head.set_pan(1.57)

		self.pause_event = Event()
		self.object_poses = None

		# time.sleep(3)
		# self.move_to_init()
		# self.perform_grasp('left')

		self.lGripper.close()
		time.sleep(3)
		if not self.lGripper.gripping():
			print "missed"
		else:
			print "didnt miss"
		time.sleep(3)
		print self.lGripper.force()
		# self.move_to_sleep()
		'''
		# self.pause_event = Event()
		# left_arm_follow_trajectory('waypoints/lift.wp', self.left_arm, self.pause_event)
		# print self.left_arm.joint_angles()
		
		
		rospy.spin()
				

if __name__=="__main__":
	
	try:
		go = Pick_and_shake()


	except rospy.ROSInterruptException: pass

