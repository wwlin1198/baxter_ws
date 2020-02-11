#! /usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import time
import tf2_ros
import tf2_geometry_msgs
import tf


def broadcast_camera_transform():
	rospy.init_node('tf_head_camera_to_camera_link')
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	static_transformStamped = geometry_msgs.msg.TransformStamped()
	rate = rospy.Rate(100)

	while not rospy.is_shutdown(): 
		static_transformStamped.header.stamp = rospy.Time.now()
		static_transformStamped.header.frame_id = 'head'
		static_transformStamped.child_frame_id = 'camera_link'

		static_transformStamped.transform.translation.x = 0.12839
		static_transformStamped.transform.translation.y = 0
		static_transformStamped.transform.translation.z = 0.12368

		quat = tf.transformations.quaternion_from_euler(0,1.32,0)
		static_transformStamped.transform.rotation.x = quat[0]
		static_transformStamped.transform.rotation.y = quat[1]
		static_transformStamped.transform.rotation.z = quat[2]
		static_transformStamped.transform.rotation.w = quat[3]

		broadcaster.sendTransform(static_transformStamped)
		rate.sleep()


if __name__=='__main__':
	try:
		broadcast_camera_transform()
	except rospy.ROSInterruptException:
		pass
#original (-1.57,-1.57 ,0)