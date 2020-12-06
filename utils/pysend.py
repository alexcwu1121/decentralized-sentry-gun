import serial
import numpy as np
import threading
import time

"""
Test script for:
    1. Encoding and sending numpy configuration arrays over serial
    2. Threaded receive from serial
"""

# global configuration states updated by receive from arduino
gun_config = np.array([[0, 0]]).T
camera_config = np.array([[0, 0]]).T
gun_ready = [0]

# commands to be sent to arduino
gun_command = np.array([[30, 0]]).T
camera_command = np.array([[30, 0]]).T
fire_gun = [1]

# Establish serial port and baud rate
ard = serial.Serial('COM3', 9600)
if(ard.isOpen() == False):
    ard.open()

# Write sends only one configuration state. Needs identifier label
def writeCamTurret(config):
    ard.write("c,{},{}:".format(config[0][0], config[1][0]).encode())

# Send sends only one configuration state. Needs identifier label.
# May be different from cam turret in future versions
def writeGunTurret(config):
    ard.write("g,{},{}:".format(config[0][0], config[1][0]).encode())

# Tell gun to shoot
def writeGun():
    ard.write("s:".encode())

# Receive takes state of all configs at once
def receive():
    line = []
    while True:
        for c in ard.read():
            if chr(c) == ':':
                # parse return from arduino
                print("".join(line))

                # Run comma delimited parse on [2:]
                parsed = "".join(line[2:]).split(',')
                if(line[0] == 'c'):
                    camera_config[0][0] = float(parsed[0])
                    camera_config[1][0] = float(parsed[1])
                elif(line[0] == 'g'):
                    gun_config[0][0] = float(parsed[0])
                    gun_config[1][0] = float(parsed[1])
                elif (line[0] == 's'):
                    gun_ready[0] = float(parsed[0])

                """
                print(camera_config)
                print(gun_config)
                print(gun_ready)
                """

                line = []
                break
            else:
                line.append(chr(c))

if __name__ == "__main__":
    # Receive thread
    t = threading.Thread(target=receive, args=())
    t.start()

    while True:
        writeCamTurret(camera_command)
        time.sleep(.1)
        writeGunTurret(gun_command)
        time.sleep(.1)
        writeGun()
        time.sleep(.1)

    #print("c:{},{}\n".format(gun_config[0][0], gun_config[1][0]).encode())