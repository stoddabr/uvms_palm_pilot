#!/usr/bin/env python3

# WS client 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from leap_socket.msg import LeapSummary

import websocket
import json
import time 

# todo review rospy_websocket_client

# todo timeout https://websocket-client.readthedocs.io/en/latest/examples.html#setting-timeout-value 
def leapWS(ws, pub):
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

        msg = LeapSummary()
        msg.confidence = hand_data['confidence']
        msg.timeVisible = hand_data['timeVisible']

        msg.gestures = [ gest['type'] for gest in msg_data['gestures'] ]
        msg.pinchStrength = hand_data['grabStrength']
        msg.grabStrength = hand_data['grabStrength']
        msg.palmNormal = Vector3(hand_data['palmNormal'][0], hand_data['palmNormal'][1], hand_data['palmNormal'][2])
        msg.palmPosition = Vector3(hand_data['palmPosition'][0], hand_data['palmPosition'][1], hand_data['palmPosition'][2])
        msg.palmVelocity = hand_data['palmVelocity']
        pub.publish(msg)

    except ValueError as e:
        print(f'invalid json: {msg}')


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
    pub = rospy.Publisher('/leap/summary', LeapSummary, queue_size=1)
    rospy.init_node('leap_socket', anonymous=True)
    rate = rospy.Rate(1)  # 10hz

    while not rospy.is_shutdown():
        try:
            leapWS(ws, pub)
        except rospy.ROSInterruptException:
            ws.close()
            pass

if __name__ == '__main__':
    ros_loop()

