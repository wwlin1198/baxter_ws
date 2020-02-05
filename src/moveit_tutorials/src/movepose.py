import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def __init__(self):
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

	#sets pose target
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1.0
	pose_target.position.x = 0.7
	pose_target.position.y = -0.05
	pose_target.position.z = 1.1
	group.set_pose_target(pose_target)


	#compute plan and visualize if successful (JUST PLAN)
	plan1 = group.plan()

	print "============ Waiting while RVIZ displays plan1..."
	rospy.sleep(5)

	# Uncomment below line when working with a real robot
	# group.go(wait=True)

