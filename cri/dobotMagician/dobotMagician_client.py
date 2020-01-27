# -*- coding: utf-8 -*-
"""Python client interface for Dobot Magician.

Adapted from [HOLD]

version 1.0
26 Jan 2020
"""

# ------------------------------------------#
# Imports                                   #
# ------------------------------------------#

import time
import numpy as np
from cri.dobotMagician.dll_files import DobotDllType as dType # Import the dobot dll
import transforms3d # For angle manipulations

# ------------------------------------------#
# Variables                                 #
# ------------------------------------------#

#Error terms
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"} #a dictionary of error terms as defined a C++ enum in 'DobotType.h file'


class dobotMagicianClient:
    """Python client interface for Dobot Magician
    """

    class CommandFailed(RuntimeError):
        pass

    class InvalidZone(ValueError):
        pass
    
    SERVER_ERROR = 0
    SERVER_OK = 1
    
    def __init__(self, port="", baudRate=115200):
        
        self.api = dType.load() #Load the dll to allow it to be used               
        # self._delay = .08
        self.connect(port, baudRate) # Connect to the Dobot Magician

    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.get_info())
        
    def __str__(self):
        return self.__repr__()
        
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def set_units(self, linear, angular):
        """Sets linear and angular units.
        """
        units_l = {'millimeters' : 1.0,
                   'meters' : 1000.0,
                   'inches' : 25.4,
                   }
        units_a = {'degrees' : 1.0,
                   'radians' : 57.2957795,
                   }
        self._scale_linear = units_l[linear]
        self._scale_angle  = units_a[angular]


    def synchCommand(self, command):
        """Executes a dobot command as a synchronous command
        """
        pass
        
    def connect(self, port, baudRate):
        """Connects to dobot magician.
        """

        state = dType.ConnectDobot(self.api, port, baudRate)[0] #Try and connect to dobot with automatic search, returns enumerate type
        
        #If connection is successful
        if (state == dType.DobotConnect.DobotConnect_NoError):
            self.connected = True
            print("Client connected to dobot Magician...")
        else:
            self.connected = False
            raise Exception ("Connection to dobot magician failed with error {}".format(CON_STR[state]))     
        
    
    def set_home_position(self, pose):
        """Sets home position
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """

        (x, y, z, qw, qx, qy, qz) = pose
        vec, theta = quat2axangle([qw, qx, qy, qz])

        if vec == np.array([0.,  0.,  1.]):
            dType.SetHOMEParams(self.api, 250, 0, 50, 0, isQueued = 1)  # Set home position
        else:
            raise Exception ("Pose has rotation about axis other than z, axis: {}".format(vec))
     
    def get_alarms(self):
        """returns a alarm identifier string
        """       
        alarms = dType.GetAlarmsState(self.api) #Get the alarms
        alarmsState = alarms[0]
        print("alarmsState: {} | {} | {} | {} | {} | {} | {} | {} ".format(alarmsState[0], alarmsState[1], alarmsState[2], alarmsState[3], alarmsState[4], alarmsState[5], alarmsState[6], alarmsState[7]))
        return alarmsState

    def perform_homing(self):
        """
        """
        #Queue homing function
        lastIndex = dType.SetHOMECmd(self.api, temp = 0, isQueued = 1)[0] # Execute the homing function. Note temp is not used by Dobot. Returned value is the last index -> "queuedCmdIndex: If this command is added to the queue, queuedCmdIndex indicates the index of this command in the queue. Otherwise, it is invalid."
        print("ReturnHoming: {}".format(lastIndex))

        #Execute commands up to homing function
        dType.SetQueuedCmdStartExec(self.api) # Start running commands in command queue 
        
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]: # Loop gets current index, and waits for the command queue to finish
            dType.dSleep(100)

        return True


    def get_info(self):
        """retvalsurns a unique robot identifier string.
        """
        # command = 0
        # sendMsg = pack('>H', command)
        # self.sock.send(sendMsg)
        # time.sleep(self._delay)
        # receiveMsg = self.sock.recv(4096)
        # retvals = unpack_from('>H', receiveMsg)
        # ack = retvals[0]
        # if ack != ABBClient.SERVER_OK:
        #     raise ABBClient.CommandFailed            
        # info = receiveMsg[calcsize('>H'):].decode()
        # return info 
        pass

    # def move_joints(self, joint_angles):
    #     """Executes an immediate move to the specified joint angles.
        
    #     pose = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """
    #     joint_angles = np.asarray(joint_angles, dtype=np.float32).ravel()
    #     joint_angles *= self._scale_angle

    #     command = 1
    #     sendMsg = pack('>Hffffff', command, *joint_angles)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  

    # def move_linear(self, pose):
    #     """Executes a linear/cartesian move from the current base frame pose to
    #     the specified pose.
        
    #     pose = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """
    #     pose = np.asarray(pose, dtype=np.float32).ravel()
    #     pose[:3] *= self._scale_linear
        
    #     command = 2
    #     sendMsg = pack('>Hfffffff', command, *pose)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed       

    # def move_circular(self, via_pose, end_pose):
    #     """Executes a movement in a circular path from the current base frame
    #     pose, through via_pose, to end_pose.
        
    #     via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """        
    #     via_pose = np.asarray(via_pose, dtype=np.float32).ravel()
    #     via_pose[:3] *= self._scale_linear
    #     end_pose = np.asarray(end_pose, dtype=np.float32).ravel()
    #     end_pose[:3] *= self._scale_linear        
        
    #     command = 3
    #     sendMsg = pack('>Hffffffffffffff', command, *via_pose, *end_pose)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  

    # def set_tcp(self, tcp):
    #     """Sets the tool center point (TCP) of the robot.
        
    #     The TCP is specified in the output flange frame, which is located at
    #     the intersection of the tool flange center axis and the flange face,
    #     with the z-axis aligned with the tool flange center axis.
        
    #     tcp = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """
    #     tcp = np.asarray(tcp, dtype=np.float32).ravel()
    #     tcp[:3] *= self._scale_linear
        
    #     command = 4
    #     sendMsg = pack('>Hfffffff', command, *tcp)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  

    # def set_work_object(self, work_object):
    #     """Sets the work object on the robot.
        
    #     The work object is a local coordinate frame on the robot, where
    #     subsequent linear moves will be in this coordinate frame. 
        
    #     work_object = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """
    #     work_object = np.asarray(work_object, dtype=np.float32).ravel()
    #     work_object[:3] *= self._scale_linear
        
    #     command = 5
    #     sendMsg = pack('>Hfffffff', command, *work_object)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed       

    # def set_speed(self, linear_speed, angular_speed):
    #     """Sets the linear speed (default mm/s) and angular speed
    #     (default deg/s) of the robot TCP.
    #     """
    #     linear_speed *= self._scale_linear
    #     angular_speed *= self._scale_angle

    #     command = 6
    #     sendMsg = pack('>Hff', command, linear_speed, angular_speed)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed     

    # def set_zone(self, 
    #              zone_key = 'z0', 
    #              point_motion = False, 
    #              manual_zone = None):
    #     zone_dict = {'z0':  (0.3, 0.3, 0.03), 
    #                  'z1':  (1, 1, 0.1), 
    #                  'z5':  (5, 8, 0.8), 
    #                  'z10': (10, 15, 1.5), 
    #                  'z15': (15, 23, 2.3), 
    #                  'z20': (20, 30, 3), 
    #                  'z30': (30, 45, 4.5), 
    #                  'z50': (50, 75, 7.5), 
    #                  'z100': (100, 150, 15), 
    #                  'z200': (200, 300, 30),
    #                 }
    #     """Sets the motion zone of the robot. This can also be thought of as
    #     the flyby zone, AKA if the robot is going from point A -> B -> C,
    #     how close do we have to pass by B to get to C
        
    #     zone_key: uses values from RAPID handbook (stored here in zone_dict)
    #     with keys 'z*', you should probably use these

    #     point_motion: go to point exactly, and stop briefly before moving on

    #     manual_zone = [pzone_tcp, pzone_ori, zone_ori]
    #     pzone_tcp: default mm, radius from goal where robot tool centerpoint 
    #                is not rigidly constrained
    #     pzone_ori: default mm, radius from goal where robot tool orientation 
    #                is not rigidly constrained
    #     zone_ori: default degrees, zone size for the tool reorientation
    #     """
    #     if point_motion: 
    #         zone = np.array((0, 0, 0))
    #     elif manual_zone is not None and len(manual_zone) == 3:
    #         zone = np.asarray(manual_zone, dtype=np.float32).ravel()
    #     elif zone_key in zone_dict.keys(): 
    #         zone = np.array(zone_dict[zone_key])
    #     else:
    #         raise ABBClient.InvalidZone
 
    #     zone[0] *= self._scale_linear
    #     zone[1] *= self._scale_linear
    #     zone[2] *= self._scale_angle

    #     command = 7
    #     sendMsg = pack('>HHfff', command, int(point_motion), *zone)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>H', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  

    # def get_joint_angles(self):
    #     """retvalsurns the robot joint angles.
        
    #     joint_angles = (j0, j1, j2, j3, j4, j5)
    #     j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
    #     measured in degrees (default)
    #     """       
    #     command = 8
    #     sendMsg = pack('>H', command)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>Hffffff', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  
    #     joint_angles = np.asarray(retvals[1:], dtype=np.float64)
    #     joint_angles /= self._scale_angle
    #     return joint_angles
    
    # def get_pose(self):
    #     """retvalsurns the TCP pose in the reference coordinate frame.
        
    #     pose = (x, y, z, qw, qx, qy, qz)
    #     x, y, z specify a Euclidean position (default mm)
    #     qw, qx, qy, qz specify a quaternion rotation
    #     """
    #     command = 9
    #     sendMsg = pack('>H', command)
    #     self.sock.send(sendMsg)
    #     time.sleep(self._delay)
    #     receiveMsg = self.sock.recv(4096)
    #     retvals = unpack_from('>Hfffffff', receiveMsg)
    #     ack = retvals[0]
    #     if ack != ABBClient.SERVER_OK:
    #         raise ABBClient.CommandFailed  
    #     pose = np.asarray(retvals[1:], dtype=np.float64)
    #     pose[:3] /= self._scale_linear      
    #     return pose
        
    def close(self):
        """Releases any resources held by the controller (e.g., sockets). And disconnects from Dobot magician
        """
        dType.DisconnectDobot(self.api) #Disconnect the Dobot
        print("Shutting down client ... Dobot disconnected !")