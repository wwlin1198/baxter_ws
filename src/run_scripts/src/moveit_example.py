#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
import baxter_interface
from math import pi, sqrt
from operator import itemgetter
from sensor_msgs.msg import Joy, Imu, JointState, Image
from baxter_core_msgs.msg import EndpointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, PoseArray
import cv2
import json
from rospy_message_converter import message_converter as mc 
import sys
import os
import copy
import random

from signal import signal, SIGINT
from sys import exit

# from positionControl import *
class moveit_example:


    def __init__(self): 
        # initialize ROS node
        self.picking = False
        rospy.init_node('eexxaammple', anonymous=True)

         # create variables for the arms

        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')

        # print self.left_arm.joint_angles()
        # i = 0;
        # while (i == 0):
        #     print(self.left_arm.joint_efforts())
        #     rospy.sleep(.5)
        
        #release_object(self)
        
        rospy.Subscriber('/object_poses', PoseArray, self.object_poses_callback)
        self.calibrated = False

        # initialize arm group variables for MoveIt
        self.both_arms_group = MoveGroupInterface("both_arms", "base")
        self.right_arm_group = MoveGroupInterface("right_arm", "base")
        self.left_arm_group = MoveGroupInterface("left_arm", "base")

        # create variables for the grippers
        self.leftgripper = baxter_interface.Gripper('left')
        self.rightgripper = baxter_interface.Gripper('right')

        # calibrate left gripper and keep it open
        self.leftgripper.calibrate()
        self.leftgripper.open()        
       
       # calibrate right gripper and keep it open
        self.rightgripper.calibrate()
        self.rightgripper.open()

        # names of the left and right joints
        self.both_arm_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']

        self.right_joints=['right_s0','right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0','right_e1']
        self.left_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
        
        self.object_poses = None


        # joint angles for initial position of arms
        # these were manually recorded. I essentially just moved the arms to the positions
        # I desired and looked up their joint angles.

        self.init_joint_angles = [0.2515728492132078, 1.58958759144626, 3.0418839023767754,  -1.2862428906419194, 
                1.2916118233995184,  0.37735927381981177, -1.3203739631723699,  -0.1514806028036846, 
                -1.2908448330055757,  2.908044078633773, -1.5424176822187836, -0.34131072530450457, 
                0.984048675428493,  1.1949710337627373]

        self.tug = [0.7294078646395142, 1.1850001586414822, 1.957359485341788, -0.9821311994436361, 1.5397332158399841, -0.7742768026851626,  -0.7566360236244803]


        # joint angles for pre-grasping poses of the left and right arms.
        # These were manually retrieved. I essentially just moved the arms to the desired
        # positions and read off their joint angles
        self.preright = [0.27381557063754636, -0.7309418454273996,  2.269908070873441,  -1.0181797479589434,  0.9177040063524488,1.1213399559442374,  1.878359474765689]
        self.preleft = [0.7294078646395142, 1.1850001586414822, 1.957359485341788, -0.9821311994436361, 1.5397332158399841, -0.7742768026851626, -0.7566360236244803]
        
        self.pregrasp_pose = [0.3996019952441503, 1.7648448964621686, 2.2507333110248733, -1.0599807244288209, 1.1293933550806359, -0.6764855274574675, -1.113286556807839]
        # Here, I manually specify the 3D position and orientation of the object.
        # I hard-code the position here. 
        # This is possibly the only functionality yout'd modify once the vision system is running.
        # Once the vision system is running, it will publish the poses of candidate objects to 
        # a ROS topic. Your job would be to to read pose messages from the topic, select a pose
        # from the list and feed it to moveit.
        # 
        self.pickgoal = PoseStamped() 
        self.pickgoal.header.frame_id = "base"
        self.pickgoal.header.stamp = rospy.Time.now()
        self.pickgoal.pose.position.x = 0.57
        self.pickgoal.pose.position.y = -0.1 
        self.pickgoal.pose.position.z = -0.05
        self.pickgoal.pose.orientation.x = 1.0
        self.pickgoal.pose.orientation.y = 0.0
        self.pickgoal.pose.orientation.z = 0.0
        self.pickgoal.pose.orientation.w = 0.0


        ######### THIS IS WHERE THE ARM MOTION HAPPENS ##############

        # Move both arms to  their initial positions. Then wait for 5 seconds (rospy.sleep(5))
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False) 
        # rospy.sleep(5)
         
      
        # # Move the left arm to the pre-grasping position so that the grippers hover over
        # # the object
        # self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)

        # # Move arm to the object
        # self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)

        # # close the left gripper
        # self.leftgripper.close()

        # self.preleft = [0.41302432713814763, 1.419315723990979, 2.2227381616459647, -0.724805922275858,
        #                 1.2198982215658754, -0.947233136519243, -0.9100341024130217]

        # Move arm back to the pre-grasping position. At this point the object should be 
        # between the grippers
        # self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)



        # self.preleft = [0.336325287743877, 1.1060001480653834, 2.5651993725413833, -0.7156020375485456,
        #                 1.7215099392044055, -0.45099035163831164, -1.2241166687325602]

        # self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)

        # self.leftgripper.open()
        rospy.sleep(3)
        for i in range(5):
            self.pick_arbitrary_object()
            rospy.sleep(2)
        rospy.spin()

    def object_poses_callback(self, data):
        if not self.picking:
            N = len(data.poses)
            if N > 0:
                self.object_poses = data.poses 
            else:
                self.object_poses = None


    def pick_arbitrary_object(self):
        if self.object_poses is not None:
            N = len(self.object_poses)
            target = random.sample(self.object_poses, 1)[0]
            self.pick_up_and_put_back(target)
            

    def pick_up_and_put_back(self, pose):
        stamped = PoseStamped()
        stamped.header.frame_id = "base"
        stamped.header.stamp = rospy.Time.now()
        stamped.pose = pose
        self.picking = True
        #pick up
        self.left_arm_group.moveToJointPosition(self.left_joints, self.pregrasp_pose, plan_only=False)
        self.left_arm_group.moveToPose(stamped, "left_gripper", plan_only=False)
        self.leftgripper.close()
        self.left_arm_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)
        rospy.sleep(2)

        #put back
        self.left_arm_group.moveToJointPosition(self.left_joints, self.pregrasp_pose, plan_only=False)
        self.left_arm_group.moveToPose(stamped, "left_gripper", plan_only=False)
        self.leftgripper.open()
        self.left_arm_group.moveToJointPosition(self.left_joints, self.pregrasp_pose, plan_only=False)
        self.left_arm_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False) 
        self.picking = False
        rospy.sleep(2)



def handler(signal_received, frame):
    print('SIGINT or CTRL-C detected. Exiting...')
    exit(0)

        
if __name__=='__main__':
    signal(SIGINT, handler)

    print('Running. Press CTRL-C to exit')

    try:
		t = moveit_example()
        

    except rospy.ROSInterruptException:
        pass

