
#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PointStamped, PoseArray, PoseStamped
from visualization_msgs.msg import MarkerArray
from math import sqrt
import tf2_ros
import tf2_geometry_msgs
import tf

class object_pose:

	def compute_center(self, point_array, header):
		mean_x = 0.0; mean_y = 0.0; mean_z = 0.0; 
		length = len(point_array)
		for p in point_array:
			mean_x += p.x 
			mean_y += p.y 
			mean_z += p.z
		mean_x /= length
		mean_y /= length
		mean_z /= length

		if -0.2 < mean_y < 0:#(sqrt((mean_x**2) + (mean_y**2) + (mean_z**2))) > 0.8 and (sqrt((mean_x**2) + (mean_y**2) + (mean_z**2))) < 2.0:
			po = PointStamped()
			po.header = header
			po.point.x = mean_x
			po.point.y = mean_y
			po.point.z = mean_z

			p = PoseStamped()
			p.pose.position.x = mean_x
			p.pose.position.y = mean_y
			p.pose.position.z = mean_z
			self.point_pub.publish(po)
			# self.transform_object_pose_to_robot_rf(p)
			return po



	def viz_array_callback(self, data):
		if len(data.markers) > 0:
			poses = []
			for marker in data.markers:
				po = self.compute_center(marker.points, marker.header)
				if po is not None:
					pose = Pose()
					pose.position = po.point
					poses.append(pose)
			if len(poses) != 0:
				posearray = PoseArray()
				for p in poses:
					posearray.poses.append(p)
				self.posearray_pub.publish(posearray)
				

	def transform_object_pose_to_robot_rf(self, object_pose):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_rgb_optical_frame',rospy.Time(0),
			rospy.Duration(1.0))
		trans_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)
		self.transposearray_pub.publish(transp_pose)
		return trans_pose



	def __init__(self):
		rospy.init_node('get_marker_pose')
		rospy.Subscriber('/tabletop/clusters', MarkerArray, self.viz_array_callback)
		self.point_pub = rospy.Publisher('/obs', PointStamped, queue_size=20)
		self.posearray_pub = rospy.Publisher('/object_poses', PoseArray, queue_size=20)
		self.transposearray_pub = rospy.Publisher('/trans_poses', PoseArray, queue_size=20)
		rospy.spin()


if __name__=='__main__':
	try:
		go = object_pose()
	except:
		pass