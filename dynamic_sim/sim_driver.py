import comms
import multiprocessing as mp
import os
import sys
from example_pub import ExamplePub
from hardware_interface import HardwareInterface

def worker(type, args=None):
    if type == 'example_pub':
        example_pub = ExamplePub()
        example_pub.run()
    elif type == 'hardware_interface':
        hardware_interface = HardwareInterface()
        hardware_interface.run()
    elif type == 'camera_motion':
        camera_motion = CameraMotion()
        camera_motion.run()

def main():
    procs = []
    try:
        p = mp.Process(target=worker, args=('example_pub',))
        procs.append(p)
        p.start()

        p = mp.Process(target=worker, args=('hardware_interface',))
        procs.append(p)
        p.start()

        # Uncomment once camera motion is implemented
        # p = mp.Process(target=worker, args=('camera_motion',))
        # procs.append(p)
        # p.start()

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