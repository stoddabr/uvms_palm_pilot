#!/usr/bin/env python3

"""
for use for the demo video 

opening/closing hand (gripStrength) opens/closes gripper (via joystick workaround)
moving down causes arm to reach down
finger rotate gesture rotates the gripper (via )

""" 

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Joy

import websocket
import json
import time 

# todo review rospy_websocket_client

# todo timeout https://websocket-client.readthedocs.io/en/latest/examples.html#setting-timeout-value 
def leapWS(ws, pubjoy, pubarm, pubhaptic):
    msg = ws.recv()
    try:
        # print(msg)
        msg_data = json.loads(msg)

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
            close_gripper(pubjoy)
            pubhaptic.publish('TimePointStreaming_Forcefield')
        elif grabStrength < 0.05:
            open_gripper(pubjoy)
            pubhaptic.publish('stop')
        # else, move arm
        else:
            # mapping from palPosition.y from 120 to 320
            # mapping to joint position 0.4 to 0
            maxJoint = 1.0  # min is 0.0
            height = 1.0 - (palmPosition.y - 120) * maxJoint/200
            height = max(min(height, maxJoint), 0.0)  # clamp
            vertical_gripper(pubarm, height)
            pubhaptic.publish('TimePointStreaming_Square')


    except ValueError as e:
        print(f'invalid json: {msg}')



def vertical_gripper(pub, height):
    # construct a joystick message that will move the gripper
    print(f'changing gripper height: {height}')
    header = Header()
    header.stamp = rospy.Time.now()
    joint_names = ['alpha/joint2', 'alpha/joint3']
    velocity = [0.0003, 0.0003]
    position = [height, height]
    effort = []
    msg = JointState(header, joint_names, position, velocity, effort)
    pub.publish(msg)

def open_gripper(pub):
    # construct a joystick message that will open the gripper
    print('opening gripper')
    msg = Joy()
    msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(msg)

def close_gripper(pub):
    # construct joystick message that will close the gripper
    print('closing gripper')
    msg = Joy()
    msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.buttons = [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(msg)


def ros_loop():
    # init leapmotion socket
    leap_uri = "ws://localhost:6437/v6.json"
    ws = websocket.WebSocket()
    ws.connect(leap_uri)
    # send listener messages to leapmotion socket
    enable_gestures_msg = '{"enableGestures: true"}'
    background_msg = '{"background": true}'
    focused_msg = '{"focused": true}'
    ws.send(enable_gestures_msg)
    ws.send(focused_msg); # listening for updates
    print(ws.recv())

    # init publisher
    pubjoy = rospy.Publisher('/seabotix/joy', Joy, queue_size=1)
    pubarm = rospy.Publisher('/seabotix/alpha/arm_control/command', JointState, queue_size=1)
    pubhaptic = rospy.Publisher('haptics', String, queue_size=1)

    rospy.init_node('leap_socket', anonymous=True)
    rate = rospy.Rate(1)  # 10hz

    while not rospy.is_shutdown():
        try:
            leapWS(ws, pubjoy, pubarm, pubhaptic)
        except rospy.ROSInterruptException:
            ws.close()
            pass

if __name__ == '__main__':
    ros_loop()

