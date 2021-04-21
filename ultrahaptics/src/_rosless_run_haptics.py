import os
import subprocess 
import signal
import time 


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

        time.sleep(1)


if __name__ == '__main__':
    test()

