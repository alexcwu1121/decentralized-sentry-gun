import comms
import multiprocessing as mp
import os
import sys
import time
from example_pub import ExamplePub
from example_camera_pub import ExampleCameraPub
from camera_motion import CameraMotion
from hardware_interface import HardwareInterface
from camera_interface import CameraInterface
from gun_motion import GunMotion

def worker(type, is_sim):
    if type == 'example_pub':
        example_pub = ExamplePub()
        time.sleep(2)
        example_pub.run(is_sim)
    elif type == 'example_camera_pub':
        example_camera_pub = ExampleCameraPub()
        time.sleep(2)
        example_camera_pub.run(is_sim)
    elif type == 'hardware_interface':
        hardware_interface = HardwareInterface()
        time.sleep(2)
        #hardware_interface.run(is_sim)
        hardware_interface.runS()
    elif type == 'camera_motion':
        camera_motion = CameraMotion()
        time.sleep(2)
        camera_motion.runSweep()
    elif type == 'camera_interface':
        camera_interface = CameraInterface()
        time.sleep(2)
        camera_interface.run(is_sim)
    elif type == 'gun_motion':
        gun_motion = GunMotion()
        time.sleep(2)
        gun_motion.run(is_sim)

def main():
    procs = []

    is_sim = False

    try:
        #p = mp.Process(target=worker, args=('example_pub', is_sim))
        #procs.append(p)
        #p.start()

        p = mp.Process(target=worker, args=('gun_motion', is_sim))
        procs.append(p)
        p.start()

        #p = mp.Process(target=worker, args=('example_camera_pub', is_sim))
        #procs.append(p)
        #p.start()

        p = mp.Process(target=worker, args=('hardware_interface', is_sim))
        procs.append(p)
        p.start()

        p = mp.Process(target=worker, args=('camera_interface', is_sim))
        procs.append(p)
        p.start()

        #p = mp.Process(target=worker, args=('camera_motion', is_sim))
        #procs.append(p)
        #p.start()

    except KeyboardInterrupt:
        print('Interrupted')
        for proc in procs:
            proc.terminate()
            proc.join()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

if __name__ == "__main__":
    mp.set_start_method('spawn')
    main()