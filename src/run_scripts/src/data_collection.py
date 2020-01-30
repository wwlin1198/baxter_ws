#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("moveit_python")
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
import baxter_interface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from sensor_msgs.msg import Joy, Imu, JointState, Image
from baxter_core_msgs.msg import EndpointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
from rospy_message_converter import message_converter as mc 
import sys
import os
import copy
import Queue as queue
import thread
import sounddevice as sd
import soundfile as sf
from positionControl import *



class Sensory_data_collection:
    def __init__(self,object_name, trialnum):
        rospy.init_node('pnp', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joycallback)
        print('starting...')

        self.calibrated = False

        self.both_arms_group = MoveGroupInterface("both_arms", "base")
        self.right_arm_group = MoveGroupInterface("right_arm", "base")
        self.left_arm_group = MoveGroupInterface("left_arm", "base")

        self.leftgripper = baxter_interface.Gripper('left')
        self.rightgripper = baxter_interface.Gripper('right')

        self.leftgripper.calibrate()
        self.leftgripper.open()

        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
       
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
        self.shake3 = [0.6412039693361029, 0.34284470609239,  2.865476111769953, -1.007825377640717, 1.3219079439602552, -0.7554855380335662, -0.8199127311247536]
        self.shake4 = [1.4580487388850858, 1.326893381520883, 1.8634031620838063, -0.9882671225951778, 1.2517283229144975, -0.6830049458059805, -0.9782962474739226]

        self.prepoke = [0.6170437719269076, 0.8969952657159956, 2.0129662889026343, -0.9119515783978784, 1.9036701577657984, -0.5215534678810406, -0.9560535260495842]
        
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

        #data saving attributes
        self.current_image = None
        self.bridge = CvBridge()
        self.current_jointstate = {}
        self.current_endpointstate = {}
        self.current_accelerometer = {}
        self.keep_saving = False

        self.accel_counter=0
        self.joint_counter=0
        self.endpoint_counter = 0
        self.pause_event = Event()

        rospy.Subscriber('/robot/accelerometer/left_accelerometer/state', 
                                                Imu, self.accel_callback)
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState,
                                                self.endpoint_callback)
        rospy.Subscriber('/robot/joint_states', JointState, self.joint_callback)

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        self.object_name = object_name
        self.trialnum = trialnum

        if not os.path.exists(os.path.dirname(self.object_name)):
            try:
                os.mkdir(self.object_name)
            except OSError as exc:
                pass

        self.q = queue.Queue()
        self.should_record = False

        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False) 
        rospy.sleep(5)
        self.perform_action_sequence('left')

        rospy.spin()

    def perform_action_sequence(self,arm):
        self.capture_image(self.trialnum, self.object_name)
        self.lift(arm)
        self.shake(arm)
        self.drop(arm)
        self.push(arm)
        rospy.sleep(10)
        self.poke(arm)
        self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False) 


    def joycallback(self, data):
        if data.buttons[3] == 1:#X
            arm = 'left'
            self.keep_saving = True
            self.capture_image('lift', self.object_name)
            self.start_recording_audio('lift', self.object_name)
            self.lift(arm)
            self.stop_recording_audio()
            self.save_data('lift', self.object_name)

        elif data.buttons[1] == 1:#B
            arm = 'left'
            self.keep_saving = True
            self.capture_image('push', self.object_name)
            self.start_recording_audio('push', self.object_name)
            self.push(arm)
            self.stop_recording_audio()
            self.save_data('push', self.object_name)

        elif data.buttons[4] == 1:#Y
            arm = 'left'
            self.keep_saving = True
            self.capture_image('drop', self.object_name)
            self.start_recording_audio('drop', self.object_name)
            self.drop(arm)
            self.stop_recording_audio()
            self.save_data('drop', self.object_name)

        elif data.buttons[0] == 1:#A
            arm = 'left'
            self.keep_saving = True
            self.capture_image('shake', self.object_name)
            self.start_recording_audio('shake', self.object_name)
            self.shake(arm)
            self.stop_recording_audio()
            self.save_data('shake', self.object_name)
    

    def record_callback(self, indata, frames, time, status):
        self.q.put(indata.copy())
 

    def start_recording_audio(self, action, object_name):
        self.should_record = True
        thread.start_new_thread(self.record_audio, (action, object_name))

    def stop_recording_audio(self):
        self.should_record = False

    def record_audio(self, action, object_name):
        try:
            name = object_name+'/baxter_'+action+"_audio_"+object_name+'_'+self.trialnum+'.wav'
            with sf.SoundFile(name, mode='x', samplerate=48000,
                              channels=1) as file:
                with sd.InputStream(samplerate=48000, 
                                    channels=1, callback=self.record_callback):
                    print('#' * 80)
                    print('recording audio...')
                    print('#' * 80)
                    while self.should_record:
                        file.write(self.q.get())
                    self.q = queue.Queue()

                    
        except KeyboardInterrupt:
            print('\nRecording finished: ' + repr('test.wav'))
            parser.exit(0)
        except Exception as e:
            print(e)


    def push(self, arm):
        # Move both arms to start state              
        # self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)    

        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.prepush, plan_only=False)
            rospy.sleep(2)
            self.keep_saving = True
            self.start_recording_audio('push', self.object_name)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.postpush1, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.postpush2, plan_only=False)
            rospy.sleep(2)
            self.stop_recording_audio()
            self.save_data('push', self.object_name)
            self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)
            rospy.sleep(5)

    def shake(self, arm):
        if arm == 'left':
            #lift
            #shake
            self.keep_saving = True
            self.start_recording_audio('shake', self.object_name)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake1, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake2, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake3, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.shake4, plan_only=False)
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.stop_recording_audio()
            self.save_data('shake', self.object_name)
            rospy.sleep(5)


    def drop(self, arm):
        self.keep_saving = True
        self.start_recording_audio('drop', self.object_name)
        self.leftgripper.open()
        rospy.sleep(5)
        self.stop_recording_audio()
        self.save_data('drop', self.object_name)

        # rospy.sleep(5)

    def poke(self, arm):
        self.leftgripper.close()
        self.left_arm_group.moveToJointPosition(self.left_joints, self.prepoke, plan_only=False)
        self.keep_saving = True
        self.start_recording_audio('poke', self.object_name)
        self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
        rospy.sleep(2)
        self.stop_recording_audio()
        self.save_data('poke', self.object_name)

        rospy.sleep(5)


    def lift(self,arm):
       
        # Move both arms to start state              
        # self.both_arms_group.moveToJointPosition(self.both_arm_joints, self.init_joint_angles, plan_only=False)


        #pick
        if arm == 'left':
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            rospy.sleep(1)
            self.keep_saving = True
            self.start_recording_audio('lift', self.object_name)
            self.left_arm_group.moveToPose(self.pickgoal, "left_gripper", plan_only=False)
            self.leftgripper.close()
            self.left_arm_group.moveToJointPosition(self.left_joints, self.preleft, plan_only=False)
            self.stop_recording_audio()
            self.save_data('lift', self.object_name)
        else:
            self.right_arm_group.moveToJointPosition(self.right_joints, self.preright, plan_only=False)
            rospy.sleep(1)
            self.right_arm_group.moveToPose(self.pickgoal, "right_gripper", plan_only=False)
            self.rightgripper.close()
        rospy.sleep(5)
        

    #data saving methods
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
            self.joint_counter+=1


    def image_callback(self, image):
        self.current_image = image 


    def capture_image(self, action, object_name):
        img = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')
        shape = img.shape
        img = img[130:shape[0]-230, 330:shape[1]-200]#y,x

        name = object_name+"/baxter_image_"+object_name+'_'+self.trialnum+'.png' 
        if cv2.imwrite(name, img):
            print('Image saved')
        else:
            print('Could not save image')


    def save_data(self, action, object_name):
        self.keep_saving = False
        accel_name = object_name+'/baxter_'+action+"_accel_"+object_name+'_'+self.trialnum+'.json'
        endpoint_name = object_name+'/baxter_'+action+"_endstate_"+object_name+'_'+self.trialnum+'.json'
        joint_name = object_name+'/baxter_'+action+"_jointstate_"+object_name+'_'+self.trialnum+'.json'
        rospy.sleep(1)

        with open(accel_name, "w") as accel_file:
            d = copy.deepcopy(self.current_accelerometer)
            json.dump(d, accel_file, indent=4)
        self.current_accelerometer = {}
        self.accel_counter=0

        with open(endpoint_name, "w") as endpoint_file: 
            e = copy.deepcopy(self.current_endpointstate)
            json.dump(e, endpoint_file, indent=4)
        self.current_endpointstate = {}
        self.endpoint_counter=0

        with open(joint_name, "w") as joint_file:
            j = copy.deepcopy(self.current_jointstate)
            json.dump(j, joint_file, indent=4)
        self.current_jointstate = {}
        self.joint_counter=0

        print('Data saved')

        # self.capture_image(action, object_name)


if __name__=='__main__':
    try:
        if len(sys.argv) != 3:
            print("Enter name of object and trial number as arguments! For example: python collect_data.py schrutebucks 1")
        else:
            object_name = sys.argv[1]
            trialnum = str(sys.argv[2])
            s = Sensory_data_collection(object_name, trialnum)

    except rospy.ROSInterruptException:
        pass