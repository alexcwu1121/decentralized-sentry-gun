import serial
import numpy as np
import threading
import time

"""
Test script for:
    1. Encoding and sending numpy configuration arrays over serial
    2. Threaded receive from serial
"""

# global configuration states
gun_config = np.array([[0, 0]]).T
camera_config = np.array([[0, 0]]).T

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
                print(line)
                line = []
                break
            else:
                line.append(chr(c))

if __name__ == "__main__":
    # Receive thread
    t = threading.Thread(target=receive, args=())
    t.start()

    while True:
        writeGunTurret(camera_config)
        #writeCamTurret(gun_config)
        #writeGun()
        time.sleep(.05)

    #print("c:{},{}\n".format(gun_config[0][0], gun_config[1][0]).encode())