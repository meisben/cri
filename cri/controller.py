# -*- coding: utf-8 -*-
"""Robot controller interface/implementations provide a common, low-level
interface to various robot arms.
"""

from abc import ABC, abstractmethod

from cri.transforms import quat2axangle, axangle2quat
from cri.abb.abb_client import ABBClient
from cri.ur.rtde_client import RTDEClient
from cri.dobotMagician.dobotMagician_client import dobotMagicianClient

class RobotController(ABC):
    """Robot controller class provides a common interface to various robot arms.
    
    Poses and coordinate frames are specified using 3D Euclidean positions
    and quaternion rotations.  This makes it easy to perform coordinate
    transformations using quaternion operations.    
    """
    def __repr__(self):      
        return "{} ({})".format(self.__class__.__name__, self.info)

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
 
    @property       
    @abstractmethod
    def info(self):
        """Returns a unique robot identifier string.
        """
        pass

    @property    
    @abstractmethod
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tool = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @tcp.setter
    @abstractmethod
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        
        The TCP is specified in the output flange frame, which is located at
        the intersection of the tool flange center axis and the flange face,
        with the z-axis aligned with the tool flange center axis.
        
        tcp = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @property
    @abstractmethod
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        pass

    @linear_speed.setter
    @abstractmethod
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        pass

    @property
    @abstractmethod
    def angular_speed(self):
        """Returns the joint speed of the robot TCP (deg/s).
        """
        pass

    @angular_speed.setter    
    @abstractmethod
    def angular_speed(self, speed):
        """Sets the joint speed of the robot TCP (deg/s).
        """
        pass

    @property    
    @abstractmethod
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        pass

    @blend_radius.setter    
    @abstractmethod
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        pass

    @property
    @abstractmethod
    def joint_angles(self):
        """Returns the robot joint angles.
        
        joint angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @property    
    @abstractmethod
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass
    
    @abstractmethod
    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        
        joint_angles = (j0, j1, j2, j3, j4, j5)
        j0, j1, j2, j3, j4, j5 are numbered from base to end effector and are
        measured in degrees
        """
        pass

    @abstractmethod
    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        
        pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @abstractmethod
    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Euclidean position (mm)
        qw, qx, qy, qz specify a quaternion rotation
        """
        pass

    @abstractmethod        
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        pass

   
class ABBController(RobotController):
    """ABB controller class implements common interface robot arms.
    
    Currently implemented using an adapted version of the Python 2 client
    example in OpenABB project (https://github.com/robotics/open_abb), which
    has been modified to run under Python 3.
    
    Poses and coordinate frames are specified using 3D Euclidean positions
    and a quaternion rotations.  This format makes it easy to perform
    coordinate transformations.
    """
    def __init__(self, ip='192.168.125.1', port=5000):
        self._ip = ip
        self._port = port
        self._client = ABBClient(ip, port)
        try:
            self.tcp = (0, 0, 0, 1, 0, 0, 0)    # base frame (quaternion)
            self.linear_speed = 20              # mm/s
            self.angular_speed = 20             # deg/s
            self.blend_radius = 0               # mm
        except:
            self._client.close()
            raise

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        return "ip: {}, port: {}, info: {}".format(
                self._ip,
                self._port,
                self._client.get_info(),
                )

    @property    
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return self._tcp

    @tcp.setter    
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        self._client.set_tcp(tcp)
        self._tcp = tcp

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self._linear_speed

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        try:
            self._angular_speed
        except AttributeError:
            self._client.set_speed(linear_speed=speed,
                                   angular_speed=20)
        else:
            self._client.set_speed(linear_speed=speed,
                                   angular_speed=self._angular_speed)
        self._linear_speed = speed

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self._angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        try:
            self._linear_speed
        except AttributeError:
            self._client.set_speed(linear_speed=20,
                                   angular_speed=speed)
        else:
            self._client.set_speed(linear_speed=self._linear_speed,
                                   angular_speed=speed)
        self._angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self._blend_radius

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        if blend_radius == 0:
            self._client.set_zone(point_motion=True,
                                  manual_zone=(blend_radius,)*3)
        else:
            self._client.set_zone(point_motion=False,
                                  manual_zone=(blend_radius,)*3)
        self._blend_radius = blend_radius

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        """
        return self._client.get_pose()

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        self._client.move_joints(joint_angles)

    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        self._client.move_linear(pose)

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        self._client.move_circular(via_pose, end_pose)

    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        self._client.close()


class RTDEController(RobotController):
    """UR RTDE controller class implements common interface to robot arms.
    
    Poses and coordinate frames are specified using 3D Euclidean positions
    and quaternion rotations.  This format makes it easy to perform
    coordinate transformations.
    """
    def __init__(self, ip='192.168.125.1'):
        self._ip = ip
        self._client = RTDEClient(ip)        
        try:   
            self.tcp = (0, 0, 0, 1, 0, 0, 0)    # base frame (quaternion)
            self.linear_accel = 500             # mm/s/s
            self.linear_speed = 20              # mm/s
            self.angular_accel = 50             # deg/s/s
            self.angular_speed = 20             # deg/s
            self.blend_radius = 0               # mm
        except:
            self._client.close()
            raise

    @property    
    def info(self):
        """Returns a unique robot identifier string.
        """
        return "ip: {}, info: {}".format(self._ip, self._client.get_info())

    @property    
    def tcp(self):
        """Returns the tool center point (TCP) of the robot.
        """
        return self._tcp

    @tcp.setter    
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        self._client.set_tcp(quat2axangle(tcp))
        self._tcp = tcp

    @property
    def linear_accel(self):
        """Returns the linear acceleration of the robot TCP (mm/s/s).
        """
        return self._linear_accel
    
    @linear_accel.setter    
    def linear_accel(self, accel):
        """Sets the linear acceleration of the robot TCP (mm/s/s).
        """
        self._client.set_linear_accel(accel)
        self._linear_accel = accel

    @property   
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        return self._linear_speed

    @linear_speed.setter    
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        self._client.set_linear_speed(speed)
        self._linear_speed = speed

    @property
    def angular_accel(self):
        """Returns the angular acceleration of the robot TCP (deg/s/s).
        """
        return self._angular_accel

    @angular_accel.setter
    def angular_accel(self, accel):
        """Sets the angular acceleration of the robot TCP (deg/s/s).
        """
        self._client.set_angular_accel(accel)
        self._angular_accel = accel

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        return self._angular_speed

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        self._client.set_angular_speed(speed)
        self._angular_speed = speed

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        return self._blend_radius

    @blend_radius.setter    
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        self._client.set_blend_radius(blend_radius)
        self._blend_radius = blend_radius

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def pose(self):
        """Returns the current base frame pose.
        """
        return axangle2quat(self._client.get_pose())  

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        self._client.move_joints(joint_angles)

    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        self._client.move_linear(quat2axangle(pose))

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        self._client.move_circular(quat2axangle(via_pose),
                                   quat2axangle(end_pose))
      
    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        return self._client.close()

class dobotMagicianController(RobotController):
    """Dobot Magician controller class implements common interface robot arms.
    
    DRAFT v1.0
    
    """
    def __init__(self, port="", baudRate = 115200):
        self._baudRate = baudRate
        self._port = port
        self._client = dobotMagicianClient(port, baudRate)
        try:
            pass
            ## self.tcp = (0, 0, 0, 1, 0, 0, 0)    # base frame (quaternion)
            # self.linear_speed = 50              # mm/s
            # self.angular_speed = 50             # deg/s
            ## self.blend_radius = 0               # mm

        except:
            self._client.close()
            raise

    def current_index(self):
        """ Returns the current movement index
        """
        return self._client.get_queued_cmd_current_index()

    def set_home_params(self, pose):
        """ Sets the pose for the home position of the robot arm
        """
        self._client.set_home_params(pose)

    def perform_homing(self):
        """ Performs the homing function and moves the arm to the home position
        """
        lastIndex = self._client.set_home_cmd()
        return lastIndex # return the last movement index of this command

    def clear_command_queue(self):
        """Clears the command queue
        """
        retVal = self._client.set_queued_cmd_clear()
        return retVal

    def start_command_queue(self):
        """ Start to execute commands in the command queue
        """
        retVal = self._client.set_queued_cmd_start_exec()
        return retVal
    
    def stop_command_queue(self):
        """ Stop executing commands in the command queue
        """
        retVal = self._client.set_queued_cmd_stop_exec()
        return retVal

    def alarms(self):
        """ Get alarms state for robot arm
        """
        return self._client.get_alarms_state()

    @property
    def info(self):
        """Returns a unique robot identifier string.
        """
        # return "ip: {}, port: {}, info: {}".format(
        #         self._ip,
        #         self._port,
        #         self._client.get_info(),
        #         )
        return self._client.get_alarms_state()

    @property    
    def tcp(self):
        # """Returns the tool center point (TCP) of the robot.
        # """
        return self._client.get_tcp()

    @tcp.setter    
    def tcp(self, tcp):
        """Sets the tool center point (TCP) of the robot.
        """
        lastIndex = self._client.set_tcp(tcp)
        self._tcp = tcp
        return lastIndex

    @property
    def linear_speed(self):
        """Returns the linear speed of the robot TCP (mm/s).
        """
        # return self._linear_speed
        return self._client.get_speed_linear

    @linear_speed.setter
    def linear_speed(self, speed):
        """Sets the linear speed of the robot TCP (mm/s).
        """
        # try:
        #     self._angular_speed
        # except AttributeError:
        #     self._client.set_speed(linear_speed=speed,
        #                            angular_speed=20)
        # else:
        #     self._client.set_speed(linear_speed=speed,
        #                            angular_speed=self._angular_speed)
        # self._linear_speed = speed
        lastIndex = self._client.set_speed_linear(speed)
        return lastIndex

    @property
    def angular_speed(self):
        """Returns the angular speed of the robot TCP (deg/s).
        """
        # return self._angular_speed
        return self._client.get_speed_angular

    @angular_speed.setter
    def angular_speed(self, speed):
        """Sets the angular speed of the robot TCP (deg/s).
        """
        # try:
        #     self._linear_speed
        # except AttributeError:
        #     self._client.set_speed(linear_speed=20,
        #                            angular_speed=speed)
        # else:
        #     self._client.set_speed(linear_speed=self._linear_speed,
        #                            angular_speed=speed)
        # self._angular_speed = speed
        lastIndex = self._client.set_speed_angular(speed)
        return lastIndex

    @property
    def blend_radius(self):
        """Returns the robot blend radius (mm).
        """
        # return self._blend_radius
        pass

    @blend_radius.setter
    def blend_radius(self, blend_radius):
        """Sets the robot blend radius (mm).
        """
        # if blend_radius == 0:
        #     self._client.set_zone(point_motion=True,
        #                           manual_zone=(blend_radius,)*3)
        # else:
        #     self._client.set_zone(point_motion=False,
        #                           manual_zone=(blend_radius,)*3)
        # self._blend_radius = blend_radius
        pass

    @property
    def joint_angles(self):
        """Returns the robot joint angles.
        """
        return self._client.get_joint_angles()

    @property
    def pose(self):
        """Returns the TCP pose in the reference coordinate frame.
        """
        return self._client.get_pose()
        

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.
        """
        lastIndex = self._client.move_joints(joint_angles)
        return lastIndex

    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.
        """
        lastIndex = self._client.move_linear(pose)
        return lastIndex
        

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        """
        # self._client.move_circular(via_pose, end_pose)
        pass

    def close(self):
        """Releases any resources held by the controller (e.g., sockets).
        """
        self._client.close()