#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, JointState, Image
from baxter_core_msgs.msg import EndpointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
from rospy_message_converter import message_converter as mc 


class save_data:
	def __init__(self):
		rospy.init_node('save_data_node')
		self.current_image = None
		self.bridge = CvBridge()
		self.current_jointstate = {}
		self.current_endpointstate = {}
		self.current_accelerometer = {}
		self.keep_saving = True

		self.accel_counter=0
		self.joint_counter=0
		self.endpoint_counter = 0

		rospy.Subscriber('/robot/accelerometer/left_accelerometer/state', 
												Imu, self.accel_callback)
		rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState,
												self.endpoint_callback)
		rospy.Subscriber('/robot/joint_states', JointState, self.joint_callback)

		rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

		rospy.sleep(3)

		# self.save_data('test','shoe')


		rospy.spin()


	def accel_callback(self, data):
		if self.keep_saving:
			self.current_accelerometer[self.accel_counter] = mc.convert_ros_message_to_dictionary(data)
			self.accel_counter+=1


	def endpoint_callback(self, data):
		if self.keep_saving:
			self.current_endpointstate[self.endpoint_counter] = mc.convert_ros_message_to_dictionary(data)
			self.endpoint_counter+=1


	def joint_callback(self, data):
		if self.keep_saving:
			self.current_jointstate[self.joint_counter] = mc.convert_ros_message_to_dictionary(data)
			self.endpoint_counter+=1


	def image_callback(self, image):
		self.current_image = image 


	def capture_image(self, action, object_name):
		img = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')
		shape = img.shape
		img = img[130:shape[0]-230, 330:shape[1]-200]#y,x

		name = 'data/'+'image_'+object_name+'_'+action+'.png'
		if cv2.imwrite(name, img):
			print('Image saved')
		else:
			print('Could not save image')


	def save_data(self, action, object_name):
		self.keep_saving = False
		accel_name = "data/accel_"+object_name+'_'+action+'.json'
		endpoint_name = "data/endstate_"+object_name+'_'+action+'.json'
		joint_name = "data/jointstate_"+object_name+'_'+action+'.json'
		
		with open(accel_name, "w") as accel_file:
			json.dump(self.current_accelerometer, accel_file, indent=4)
		self.current_accelerometer = {}

		with open(endpoint_name, "w") as endpoint_file:	
			json.dump(self.current_endpointstate, endpoint_file, indent=4)
		self.current_endpointstate = {}

		with open(joint_name, "w") as joint_file:
			json.dump(self.current_jointstate, joint_file, indent=4)
		self.current_jointstate = {}

		self.capture_image(action, object_name)



# if __name__ == '__main__':
# 	s = save_data()
