### Robotiq2FingerGripper

Code for controlling the Robotiq 2F-85 gripper via serial communication and Robot Raconteur

# Code overview:

**gripper_test_script_serialcom.py**

Python example controlling the gripper via serial communication.
Notes: run with numerical input argument to specify comm port (otherwise default value is used). Position is specified as a fraction of 255 with 0 being fully opened and 255 being fully closed. Speed and force are also specified as fractions of 255, with 0 being minimum speed/force and 255 being maximum speed/force. Read current is in mA. See Robotiq manual for rough conversions to units. Rely on custom calibration curves to extract more accurate readings. 

**gripper_serialcom_timing_test.py**

Python script running various timing tests and saving data as .csv files. Use this file to estimate serial communication time requirements. Porting this code to cpp may speed up communication. 

**gripper_2finger_RR.py**

Robot Raconteur script for controlling gripper. See notes on * *gripper_test_script_serialcom.py* * for information regarding specified com port and units of commanded/read values.

**gripper_controller.robdef**

Robot Raconteur robdef file. This file must be in the same directory as * *gripper_2finger_RR.py* * to run.

**RR_timing_test.m**

Matlab script for conducting the timing test over Robot Raconteur. Data saved as .mat files.
