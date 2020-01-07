#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from sensor_msgs.msg import Joy

# arm = 'left'

class Sensory_data_collection:
    def __init__(self):
        self.calibrated = False

        self.both_arms_group = MoveGroupInterface("both_arms", "base")
        self.right_arm_group = MoveGroupInterface("right_arm", "base")
        self.left_arm_group = MoveGroupInterface("left_arm", "base")

        self.leftgripper = baxter_interface.Gripper('left')
        self.rightgripper = baxter_interface.Gripper('right')

        self.leftgripper.calibrate()
        self.leftgripper.open()
       
        self.rightgripper.calibrate()
        self.rightgripper.open()

        self.both_arm_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']

        self.right_joints=['right_s0','right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0','right_e1']
        self.left_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
        
        self.init_joint_angles = [0.2515728492132078, 1.58958759144626, 3.0418839023767754,  -1.2862428906419194, 
                1.2916118233995184,  0.37735927381981177, -1.3203739631723699,  -0.1514806028036846, 
                -1.2908448330055757,  2.908044078633773, -1.5424176822187836, -0.34131072530450457, 
                0.984048675428493,  1.1949710337627373]

        self.preright = [0.27381557063754636, -0.7309418454273996,  2.269908070873441,  -1.0181797479589434,  0.9177040063524488,1.1213399559442374,  1.878359474765689]
        self.preleft = [0.7294078646395142, 1.1850001586414822, 1.957359485341788, -0.9821311994436361, 1.5397332158399841, -0.7742768026851626, -0.7566360236244803]



    def joycallback(self, data):
        if data.buttons[3] == 1:
            arm = 'left'
            self.picknplace(arm)
        elif data.buttons[1] == 1:
            arm = 'right'
            #picknplace(arm)
        elif data.buttons[4] == 1:
            arm = 'left'
            self.drop(arm)
        elif data.buttons[0] == 1:
            arm = 'left'
            #push


    def drop(self, arm):
        # leftLimb = baxter_interface.Limb('left')
        # rightLimb = baxter_interface.Limb('right')
        # print leftLimb.joint_angles()
        # print " "
        # print rightLimb.joint_angles()

        # '''
        
        # Clear planning scene
        p.clear()
        # Add table as attached object
        # p.attachBox('table', 0.43,0.67,0.43, 0.57, 0.0, -0.1, 'base', touch_links=['pedestal'])

        # Move both arms to start state              
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

        pickgoal = PoseStamped() 
        pickgoal.header.frame_id = "base"
        pickgoal.header.stamp = rospy.Time.now()
        pickgoal.pose.position.x = 0.57
        pickgoal.pose.position.y = -0.1 
        pickgoal.pose.position.z = -0.05
        pickgoal.pose.orientation.x = 1.0
        pickgoal.pose.orientation.y = 0.0
        pickgoal.pose.orientation.z = 0.0
        pickgoal.pose.orientation.w = 0.0

        # arm = 'right'

        #pick
        if arm == 'left':
            gl.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
            rospy.sleep(3)
            gl.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.leftgripper.open()
        else:
            gr.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            gr.moveToPose(pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.close()
            rospy.sleep(3)
            gr.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            self.rightgripper.open()

        rospy.sleep(1)
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)


    def picknplace(arm):
        # Define initial parameters

        if not self.calibrated:
            self.leftgripper.calibrate()
            self.leftgripper.open()
           
            self.rightgripper.calibrate()
            self.rightgripper.open()
            self.calibrated = True

        # leftLimb = baxter_interface.Limb('left')
        # rightLimb = baxter_interface.Limb('right')
        # print leftLimb.joint_angles()
        # print " "
        # print rightLimb.joint_angles()

        # '''
        self.both_arm_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']

        self.right_joints=['right_s0','right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0','right_e1']
        self.left_joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
        
        self.init_joint_angles = [0.2515728492132078, 1.58958759144626, 3.0418839023767754,  -1.2862428906419194, 
                1.2916118233995184,  0.37735927381981177, -1.3203739631723699,  -0.1514806028036846, 
                -1.2908448330055757,  2.908044078633773, -1.5424176822187836, -0.34131072530450457, 
                0.984048675428493,  1.1949710337627373]

        self.preright = [0.27381557063754636, -0.7309418454273996,  2.269908070873441,  -1.0181797479589434,  0.9177040063524488,1.1213399559442374,  1.878359474765689]
        self.preleft = [0.7294078646395142, 1.1850001586414822, 1.957359485341788, -0.9821311994436361, 1.5397332158399841, -0.7742768026851626, -0.7566360236244803]

        # Clear planning scene
        p.clear()
        # Add table as attached object
        # p.attachBox('table', 0.43,0.67,0.43, 0.57, 0.0, -0.1, 'base', touch_links=['pedestal'])

        # Move both arms to start state              
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

        pickgoal = PoseStamped() 
        pickgoal.header.frame_id = "base"
        pickgoal.header.stamp = rospy.Time.now()
        pickgoal.pose.position.x = 0.57
        pickgoal.pose.position.y = -0.1 
        pickgoal.pose.position.z = -0.05
        pickgoal.pose.orientation.x = 1.0
        pickgoal.pose.orientation.y = 0.0
        pickgoal.pose.orientation.z = 0.0
        pickgoal.pose.orientation.w = 0.0

        # arm = 'right'

        #pick
        if arm == 'left':
            gl.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
        else:
            gr.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            gr.moveToPose(pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.close()

        rospy.sleep(1)
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)
        rospy.sleep(1)
        #place
        if arm == 'left':
            gl.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            gl.moveToPose(pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.open()
            gl.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

        else:
            gr.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            gr.moveToPose(pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.open()
            gr.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

    

if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        rospy.Subscriber('/joy', Joy, joycallback)
        # picknplace()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass