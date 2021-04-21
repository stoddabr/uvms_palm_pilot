#!/usr/bin/env python3

"""
for use for the demo video 

opening/closing hand (gripStrength) opens/closes gripper (via joystick workaround)
moving down causes arm to reach down

""" 

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Joy
from leap_socket.msg import LeapSummary

import websocket
import json
import time 


class PalmPilot(object):
    """ maps motion from a hand to a robot UVMS for teleop grasping


    contains both a subscriber and publisher node so setup is inspired from this forum answer
        https://answers.ros.org/question/53394/writing-a-python-node-that-publishes-and-subscribes-to-topics-of-different-message-frequency/
    """


    def __init__(self):
        # setup node
        rospy.init_node('uvms_palm_pilot', anonymous=True)

        # init publishers
        self.pubjoy = rospy.Publisher('/seabotix/joy', Joy, queue_size=1)
        self.pubarm = rospy.Publisher('/seabotix/alpha/arm_control/command', JointState, queue_size=1)
        self.pubhaptic = rospy.Publisher('haptics', String, queue_size=1)

        # subscribe to leapmotion hand information
        rospy.Subscriber('/leap/summary', LeapSummary, self.mapMotion, queue_size=1)

    def run(self):
        """ spins up the ros node """
        r = rospy.Rate(1)
        rospy.spin()     

    def mapMotion(self, msg_data):
        """ map hand motion to robot """
        print("Recieving hand data...")

        # TODO parse as msg classes
        # OR convert to dictionary https://github.com/uos/rospy_message_converter git

        # check if msg is not hand data
        if not 'hands' in msg_data:
            rospy.loginfo(f'no hand in data message: {msg}')
            return

        num_hands = len(msg_data['hands'])
        if num_hands == 0:
            rospy.loginfo('no hands found')
            return
        if num_hands > 1:
            rospy.logwarn(f'{num_hands} hands found. Showing data for the first one')
        hand_data = msg_data['hands'][0]

        confidence = hand_data['confidence']
        timeVisible = hand_data['timeVisible']

        gestures = [ gest['type'] for gest in msg_data['gestures'] ]
        pinchStrength = hand_data['grabStrength']
        grabStrength = hand_data['grabStrength']
        palmNormal = Vector3(hand_data['palmNormal'][0], hand_data['palmNormal'][1], hand_data['palmNormal'][2])
        palmPosition = Vector3(hand_data['palmPosition'][0], hand_data['palmPosition'][1], hand_data['palmPosition'][2])
        palmVelocity = hand_data['palmVelocity']

        # if gipping
        print(f'grap strength {grabStrength}')

        if grabStrength > 0.95:
            self._set_gripper_close(pubjoy)
            pubhaptic.publish('TimePointStreaming_Forcefield')
        elif grabStrength < 0.05:
            self._set_gripper_open(pubjoy)
            pubhaptic.publish('stop')
        # else, move arm
        else:
            # mapping from palPosition.y from 120 to 320
            # mapping to joint position 0.4 to 0
            maxJoint = 1.0  # min is 0.0
            height = 1.0 - (palmPosition.y - 120) * maxJoint/200
            height = max(min(height, maxJoint), 0.0)  # clamp
            self._set_gripper_vertical(pubarm, height)
            pubhaptic.publish('TimePointStreaming_Square')

    def _set_gripper_vertical(self, height):
        # construct a joystick message that will move the gripper vertically
        print(f'changing gripper height: {height}')
        header = Header()
        header.stamp = rospy.Time.now()
        joint_names = ['alpha/joint2', 'alpha/joint3']
        velocity = [0.0003, 0.0003] # reasonable speed
        position = [height, height]
        effort = []
        msg = JointState(header, joint_names, position, velocity, effort)
        pub.publish(msg)

    def _set_gripper_open(self):
        # construct a joystick message that will open the gripper
        print('opening gripper')
        msg = Joy()
        msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        pub.publish(msg)

    def _set_gripper_close(self):
        # construct joystick message that will close the gripper
        print('closing gripper')
        msg = Joy()
        msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        pub.publish(msg)


if __name__ == '__main__':
    pp = PalmPilot()
    print('running UVMS PalmPilot')
    pp.run()

