# Connect to Robotiq 2-finger gripper via Robot Racontuer

# NOTE: This script accepts a single argument as the communication port.
# If a number is not provided, the default value will be used.

import serial
import time
import binascii
import sys
import RobotRaconteur as RR
import thread
import threading
import os

def crc_comp(data_str):
    # Generate cyclic redundancy check for modbus RTU protocol
    # Inputs:
        # data_str: data message as a string (in hex)
    # Outputs:
        # crc: cyclic redundancy check as a string (in hex)
        # NOTE: message must be re-arranged so that lowest byte is sent first
    data = bytearray.fromhex(data_str)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if ((crc & 1) !=0):
                crc >>=1
                crc ^= 0xA001
            else:
                crc >>=1
    return("%04X"%(crc))

def activate(gripper,gripper_connected):
    # This function activates the gripper and moves the gripper to an
    # open position.
    # Inputs:
        # gripper: gripper serial communication
        # gripper_connected: boolean checking that connection was successful
        #
    # Outputs:
        # gripper_activated: boolean stating whether activation was successful or not
    if gripper_connected:
        print("Activating gripper...")
        gripper.write(bytearray.fromhex("091003E80003060000000000007330")) # clear rAct
        time.sleep(0.01)
        gripper.write(bytearray.fromhex("091003E800030601000000000072E1")) # activate 
        time.sleep(0.01)
        data = gripper.readline() # read entire line to clear all data

        # Wait until activation has completed
        timeout = 10
        gripper_activated = False
        t_start = time.time()
        t = time.time()-t_start
        while (t<timeout)&(not gripper_activated):
            gripper.write(bytearray.fromhex("090307D0000185CF")) # read data
            data = binascii.hexlify(gripper.readline())
            if int(data[6])==3:
                gripper_activated = True
                print("Gripper activated!")
            time.sleep(0.5)
            t = time.time()-t_start
        if not gripper_activated:
            print("Activation timed out.")
        return gripper_activated
    else:
        gripper_activated = False
        return gripper_activated

class gripper_imp:
    # Object implementation of gripper control
    def __init__(self,gripper):
        self._lock = threading.RLock()
        self._gripper = gripper
    def setPosition(self, position, speed, force):
        # Set gripper position with specified speed and force
        # Inputs:
            # position: gripper position as a fraction of 255 (0 open, 255 closed)
            # speed: gripper speed as a fraction of 255 (0 slowest, 255 fastest)
            # force: gripper force as a fraction of 255 (0 min force, 255 max force)
        with self._lock:
            position = round(position)
            speed = round(speed)
            force = round(force)
            if position<0:
                position = 0
            elif position>255:
                position = 255
            if speed<0:
                speed = 0
            elif speed>255:
                speed = 255
            if force<0:
                force = 0
            elif force>255:
                force = 255
            position = int(position)
            speed = int(speed)
            force = int(force)

            command = "091003E8000306090000"+hex(position)[2:].zfill(2) + hex(speed)[2:].zfill(2) + hex(force)[2:].zfill(2)
            crc = crc_comp(command)
            command = command + crc[2]+crc[3]+crc[0]+crc[1]
            self._gripper.write(bytearray.fromhex(command))
            data = self._gripper.read(8)
    def getPosition(self):
        # Read current gripper position
        # Outputs:
            # pos: position as fraction of 255 (0 fully open, 255 closed)
        with self._lock:
            self._gripper.write(bytearray.fromhex("090307D00003040E"))
            data = binascii.hexlify(self._gripper.read(11))
            pos = int(data[14]+data[15],16)
            return pos
    def getCurrent(self):
        # Read current gripper current
        # Outputs:
            # current: current in mA
        with self._lock:
            self._gripper.write(bytearray.fromhex("090307D00003040E"))
            data = binascii.hexlify(self._gripper.read(11))
            current = 10*int(data[16]+data[17],16)
            return current

def main():
    # Specify comm port:
    comm_default = 3
    if len(sys.argv)>1:
        if sys.argv[1].isdigit():
            print"Comm port specified: ",str(sys.argv[1])
            comm_port = sys.argv[1]
        else:
            print"Default comm port used: ",str(comm_default)
            comm_port = comm_default
    else:
        print"Default comm port used: ",str(comm_default)
        comm_port = comm_default

    # Connect to gripper:
    print("Connecting to gripper...")
    try:
        gripper = serial.Serial(port='COM'+str(comm_port),
            baudrate=115200, bytesize=8,parity='N',stopbits=1,timeout=1)
        gripper_connected = True
    except:
        print("Error connecting to gripper.")
        gripper_connected = False
    # Activate gripper:
    if gripper_connected:
        gripper_activated = activate(gripper,gripper_connected)

    # Connect Via Robot Raconteur:
    if gripper_connected&gripper_activated:
        RR.RobotRaconteurNode.s.NodeName = "GripperController"
        gripperController = gripper_imp(gripper)
        # set gripper to open position:
        gripperController.setPosition(0, 50, 50)
        time.sleep(4)

        t = RR.TcpTransport()
        t.StartServer(6006)
        RR.RobotRaconteurNode.s.RegisterTransport(t)

        cwd = os.getcwd()
        try:
            with open(cwd+'\gripper_controller.robdef','r') as f:
                service_def = f.read()
        except Exception as e:
            print("error1")
            print(e)

        try:
            RR.RobotRaconteurNode.s.RegisterServiceType(service_def)
        except Exception as e:
            print("error2")
            print(e)
        try:
            RR.RobotRaconteurNode.s.RegisterService("gripcon","edu.rpi.gripper.gripcon",gripperController)
            print("Connect at tcp://localhost:6006/GripperController/gripcon")
            raw_input("press enter to quit...\r\n")
        except Exception as e:
            print("error3")
            print(e)
    # Shutdown:
        print("Shutting down...")
        RR.RobotRaconteurNode.s.Shutdown()
        gripperController._gripper.close()
        print("Shutdown complete!")

if __name__=='__main__':
    main()
