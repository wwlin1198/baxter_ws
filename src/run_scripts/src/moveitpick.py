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
        rospy.init_node('pnp', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joycallback)
        

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

        self.prepush = [0.6914418401393503, 0.877820505867428,  0.5725583290782307, -0.7827136970185323, 1.7636944108712544, -0.6354515413815326, -0.7646894227608787]
        self.postpush1 = [0.5767767762449155, 1.0749370371107037, 0.5591359971842333,  -0.68568941218478,  1.4166312576121796, -0.9744612955042091, -0.6653641667452982]
        self.postpush2 = [0.5556845404114912, 1.135912773429149, 0.4275971446230591, -0.656543777214957, 1.0020729496861465, -1.23255356306593, -0.4686311306989939]

        self.shake1 = [0.7232719414879726, 1.1455001533534328, 0.9177040063524488, -0.9652574107768966,  1.551621566946096, -0.7735098122912198, -0.7631554419729933]
        self.shake2 = [0.6753350418665534, 1.016645767171058, 2.992413021967471, -0.9269078910797612, 1.6712720684011584, -0.8011214664731573, -0.8084078752156131]

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

        rospy.spin()



    def joycallback(self, data):
        if data.buttons[3] == 1:#X
            arm = 'left'
            self.lift(arm)
        elif data.buttons[1] == 1:#B
            arm = 'left'
            self.push(arm)
        elif data.buttons[4] == 1:#Y
            arm = 'left'
            self.drop(arm)
        elif data.buttons[0] == 1:#A
            arm = 'left'
            self.shake(arm)


    def push(self, arm):
        # Move both arms to start state              
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)    

        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.prepush, plan_only=False)
            rospy.sleep(2)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.postpush1, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.postpush2, plan_only=False)
            rospy.sleep(2)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

    def shake(self, arm):
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)    
        if arm == 'left':
            #lift
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
            rospy.sleep(3)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)

            #shake
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake1, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake2, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake1, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake2, plan_only=False)

            #place down
            rospy.sleep(2)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.open()

            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)    



    def drop(self, arm):

        # Move both arms to start state              
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)    

        #pick
        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
            rospy.sleep(3)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.leftgripper.open()
        else:
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            self.right_arm_group.moveToPose(self.pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.close()
            rospy.sleep(3)
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            self.rightgripper.open()

        rospy.sleep(1)
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)



    def lift(self,arm):
       
        # Move both arms to start state              
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)


        #pick
        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
        else:
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            self.right_arm_group.moveToPose(self.pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.close()

        rospy.sleep(1)
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)
        rospy.sleep(1)

        #place
        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.open()
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

        else:
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            self.right_arm_group.moveToPose(self.pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.open()
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)

    

if __name__=='__main__':
    try:
        s = Sensory_data_collection()

    except rospy.ROSInterruptException:
        pass