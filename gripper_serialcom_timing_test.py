# Robotiq 2-Finger Gripper Timing Test Script

# This script tests command send and read data execution times
# and saves the results as .csv files
# Four files are saved:
    # rand_cmd.csv: 20 data points for random position, speed, and force commands
    # rand_cmd_pos.csv: 20 data points for random position, fixed speed and force commands
    # inc_cmd.csv: large number of data points for incrementally specified position at the same speed and force
    # inc_read.csv: large number of data points for reading position and current (includes position and current data)
        # 1st row: timing data
        # 2nd row: position data
        # 3rd row: current data
import serial
import time
import binascii
import sys
import random
import csv

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

def readPosCurrent(gripper):
    # Read current gripper state
    # Inputs:
        # gripper: gripper serial communication
    # Outputs:
        # [pos, current] = position and current data, respectively
        # Note: position data is a fraction of 255, where 0 corresponds
            # to fully open and 255 corresponds to fully closed
            # Current is in mA
    gripper.write(bytearray.fromhex("090307D00003040E"))
    data = binascii.hexlify(gripper.read(11)) # read specific number of bytes for this message
    # NOTE: using readline() will be slow
    # gripper does not send newline command so readline() will wait until timeout
    pos = int(data[14]+data[15],16)
    current = 10*int(data[16]+data[17],16)
    return [pos,current]
    

def setPos(gripper,pos,speed,force):
    # Set gripper position at certain speed and force
    # Inputs:
        # gripper: gripper serial communication
        # pos: position as a fraction of 255
        # speed: speed as a fraction of 255 (0 is slowest setting, 255 is full speed)
        # force: force as a fraction of 255 (0 is lowest setting, 255 is max force)
    # Make sure values are between 0 and 255:
    speed = round(speed)
    force = round(force)
    if speed<0:
        speed = 0
    elif speed>255:
        speed = 255
    if force<0:
        force = 0
    elif force>255:
        force = 255
    pos = int(pos)
    speed = int(speed)
    force = int(force)

    command = "091003E8000306090000"+hex(pos)[2:].zfill(2) + hex(speed)[2:].zfill(2) + hex(force)[2:].zfill(2)
    # Generate crc and add to the command message
    crc = crc_comp(command)
    command = command + crc[2]+crc[3]+crc[0]+crc[1]
    gripper.write(bytearray.fromhex(command))
    data = gripper.read(8)

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

    if gripper_connected:
        # Activate gripper
        gripper_activated = activate(gripper,gripper_connected)

    # Test Code
    if gripper_connected&gripper_activated:

        # random spec timing test
        print("Starting random command test...")
        timing_dat = []
        for x in range(20):
            pos = random.randint(0,256)
            speed = random.randint(0,256)
            force = random.randint(0,256)
            t_start = time.time()
            setPos(gripper,pos,speed,force)
            t = time.time()-t_start
            timing_dat.append(t)
            time.sleep(2)
            print("Complete: "+str(x+1)+" of 20")

        datafile = open('rand_cmd.csv','w+')
        with datafile:
            writer = csv.writer(datafile)
            writer.writerow(timing_dat)
        datafile.close()

        # random position spec with fixed speed and force timing test
        print("Starting random position only test...")
        timing_dat = []
        for x in range(20):
            pos = random.randint(0,256)
            t_start = time.time()
            setPos(gripper,pos,150,150)
            t = time.time()-t_start
            timing_dat.append(t)
            time.sleep(2)
            print("Complete: "+str(x+1)+" of 20")

        datafile = open('rand_cmd_pos.csv','w+')
        with datafile:
            writer = csv.writer(datafile)
            writer.writerow(timing_dat)
        datafile.close()
            
        # incremental spec timing test
        print("Starting incremental tests...")
        setPos(gripper,0,150,150)
        timing_dat_cmd = []
        timing_dat_read = []
        time.sleep(5)
        for x in range(256):
            t_start = time.time()
            setPos(gripper,x,150,150)
            t = time.time()-t_start
            timing_dat_cmd.append(t)
            time.sleep(0.01)
            
            t_start = time.time()
            readDat = readPosCurrent(gripper)
            t = time.time()-t_start
            timing_dat_read.append([t,readDat[0],readDat[1]])
            time.sleep(0.01)

        datafile = open('inc_cmd.csv','w+')
        with datafile:
            writer = csv.writer(datafile)
            writer.writerow(timing_dat_cmd)
        datafile.close()

        datafile = open('inc_read.csv','w+')
        with datafile:
            writer = csv.writer(datafile)
            writer.writerows(timing_dat_read)
        datafile.close()
            
            
        gripper.close()
    elif gripper_connected:
        gripper.close()
    

if __name__=='__main__':
    main()
