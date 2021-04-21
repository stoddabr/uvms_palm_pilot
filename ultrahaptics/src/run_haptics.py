import rospy
import rospkg
from std_msgs.msg import String

import websocket
import json
import time 
import os
import subprocess 
import signal
import time 

class Ultrahaptics():
    def __init__(self):
        rospy.init_node('haptics_listener', anonymous=True)
        # listen to haptic channel
        rospy.Subscriber("haptics", String, self.haptics_callback)
        self.is_on = False
        self.process = None
        self.script = ''

    def haptics_callback(self, msg):
        haptic_command = msg.data

        if haptic_command == 'stop':
            self.stop_haptics()    
        elif self.script != haptic_command: # ensure command is new
            self.stop_haptics()    
            self.start_haptics(haptic_command)

    def start_haptics(self, script):
        print('starting '+script)
        rp = rospkg.RosPack()
        script_path = os.path.join(rp.get_path("ultrahaptics"), "src", "haptics", script)
        self.process = subprocess.Popen("exec " + script_path, stdout=subprocess.PIPE, shell=True)
        self.is_on = True
        self.script = script

    def stop_haptics(self):
        print('stopping')
        if self.process != None:
            self.process.kill()
            self.process = None
        self.is_on = False

def test():
    while True:
        # run
        print('running')
        fname = "AmplitudeModulation_Dial_2"
        cmd = "./haptics/" + fname
        proc = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
        time.sleep(5)

        # end
        print('killing')
        proc.kill() 
        proc.terminate()

        time.sleep(1)


if __name__ == '__main__':
    ultra = Ultrahaptics()
    rospy.on_shutdown(ultra.stop_haptics)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
