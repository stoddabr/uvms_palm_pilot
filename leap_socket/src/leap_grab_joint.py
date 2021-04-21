#!/usr/bin/env python3

# WS client 

import rospy
from std_msgs.msg import String
import websocket
import json
import time 
from sensor_msgs.msg	 import JointState
from sensor_msgs.msg import Joy

"""
# todo review rospy_websocket_client

# todo timeout https://websocket-client.readthedocs.io/en/latest/examples.html#setting-timeout-value 
def leapWS(ws, pubjoy, pubhaptic):
    msg = ws.recv()
    try:
        # print(msg)
        msg_data = json.loads(msg)

        # check if msg is not hand data
        if not 'hands' in msg_data:
            print(f'handless data: {msg}')
            return

        num_hands = len(msg_data['hands'])
        if num_hands == 0:
            # print('no hands found')
            return
        if num_hands > 1:
            print(f'{num_hands} hands found. Returning first one')
        hand_data = msg_data['hands'][0]
        grabStrength = hand_data['grabStrength']
        print(f'grabst: {grabStrength}')
        
        msg = ''
        if grabStrength > 0.9:
            msg = close_msg()
            pubhaptic.publish('TimePointStreaming_Forcefield')
        elif grabStrength < 0.1:
            msg = open_msg()
            pubhaptic.publish('TimePointStreaming_Forcefield')
        
        if msg != '':
            pubjoy.publish(msg)
        else:
            pubhaptic.publish('AmplitudeModulation_Focus')

    except ValueError as e:
        print(f'invalid json: {msg}')

def open_msg():
    # construct a joystick message that will open the gripper
    print('opening')
    header = Header()
    header.stamp = rospy.Time.now()
    joint_names = []
    velocity = [0.0003]
    position = [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    msg = JointState(header, joint_names, position, velocity, effort)
    return msg


def close_msg():
    # construct joystick message that will close the gripper
    print('closing')
    msg = JointState()
    msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.buttons = [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    return msg


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
    pubhaptic = rospy.Publisher('haptics', String)
    rospy.init_node('leap_forward', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pubjoy.publish(open_msg())
    while not rospy.is_shutdown():
        try:
            leapWS(ws, pubjoy, pubhaptic)
        except rospy.ROSInterruptException:
            pass
"""
def ros_loop_keyboard():
    pub = rospy.Publisher('/seabotix/joy', Joy, queue_size=1)
    rospy.init_node('leap_forward', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pub.publish(open_msg())
    while not rospy.is_shutdown():
        try:
            print('o to open: p to close')
            keypress = input()
            print(f'got {keypress}')
            if keypress == 'o':
                pub.publish(open_msg())
            elif keypress == 'p':
                pub.publish(close_msg())
            else:
                print('not recognized')
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        # ros_loop()
        ros_loop_keyboard()
    except rospy.ROSInterruptException:
        ws.close()
        pass


