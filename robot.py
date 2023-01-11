import rospy
import time

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import math

class robot:
    __base_ang_spd = 0.8
    __base_lin_spd = 0.4
    __tgt_pt = np.zeros(2)
    __lid_sense = 10
    __rad = 0.0
    __alpha = 0.0
    __center = np.zeros(2)
    __speed = np.zeros(2)
    __center_offset = np.zeros(2)
    __tgt_speed = np.zeros(2)
    __new_data = False
    __odom_vel_log = [[]]
    __tgt_vel_log = [[]]
    __odom_log = [[]]
    __odom_is_loging = False

    def __init__(self, rad):
        self.rad = rad
        self.__odom_log.clear()
        self.__odom_vel_log.clear()
        self.__tgt_vel_log.clear()


    def calc_tgt_speed(self):
        if self.__new_data:
            self.__new_data = False
            _speed = np.zeros(2)
            tgt_angle = self.get_min_angle()
            _dist_on_tgt = np.linalg.norm(self.__center - self.__tgt_pt)

            if tgt_angle > math.pi / 80:
                _speed[1] = self.__base_ang_spd
            elif tgt_angle < -math.pi / 80:
                _speed[1] = -self.__base_ang_spd
            else:
                _speed[1] = 0  

            if abs(tgt_angle) < math.pi/8:
                _speed[0] = self.__base_lin_spd
            else:
                _speed[0] = 0

            # if tgt_angle > math.pi / 4:
            #     _speed[1] = self.__base_ang_spd
            # elif tgt_angle < -math.pi / 4:
            #     _speed[1] = -self.__base_ang_spd
            # else:
            #     if abs(tgt_angle) > math.pi / 80:
            #         _speed[1] = 1.1*tgt_angle / (_dist_on_tgt / self.__base_lin_spd) 
            #         _speed[0] = self.__base_lin_spd
            #     else:
            #         _speed[1] = 0
            #         _speed[0] = self.__base_lin_spd

            self.__tgt_speed = np.asarray(_speed)
            if self.__odom_is_loging:
                self.__tgt_vel_log.append(_speed)
                self.__odom_vel_log.append(self.__speed.tolist())
            #print(str())

    def get_min_angle(self):
        output = math.atan2(self.__tgt_pt[1] - self.__center[1], self.__tgt_pt[0] - self.__center[0]) - self.__alpha
        if output > math.pi:
            output -= 2*math.pi
        if output < -math.pi:
            output += 2*math.pi
        return output

    def set_zero_offset(self, x, y):
        self.__center_offset[0] = x
        self.__center_offset[1] = y
    
    @property
    def zero_offset(self):
        return self.__center_offset

    @property
    def center(self):
        return self.__center
    
    
    def set_center(self, x, y):
        self.__center = np.asarray([x, y])
        self.__center += self.__center_offset
        self.__new_data = True
        if self.__odom_is_loging:
            self.__odom_log.append([x+self.__center_offset[0], y+self.__center_offset[1]])

    def set_odom_speed(self, lin, ang):
        self.__speed[0] = lin
        self.__speed[1] = ang
    
    @property
    def odom_log(self):
        #return np.asarray(self.__odom_log)
        return self.__odom_log

    @property
    def alpha(self):
        return self.__alpha
    
    @alpha.setter
    def alpha(self, alpha):
        if alpha > math.pi:
            self.__alpha = alpha - math.pi * 2
        elif alpha < -math.pi:
            self.__alpha = alpha + math.pi * 2
        else:
            self.__alpha = alpha

    @property
    def tgt_point(self):
        return self.__tgt_pt    
    
    def set_tgt_point(self, x, y):
        self.__tgt_pt = np.asarray([x, y])

    def getSpeed(self):
        lin_speed = self.__tgt_speed[0]
        ang_speed = self.__tgt_speed[1]
        return lin_speed, ang_speed

    def is_arrive(self):
        vect = np.asarray(self.__center - self.__tgt_pt)
        if(math.sqrt(math.pow(vect[0], 2) +  math.pow(vect[1], 2))) < 0.1:
            return True
        else:
            return False

    def stop(self):
        self.__tgt_pt = self.__center
        self.__tgt_speed = np.zeros(2)

    def begin_odom_loging(self):
        self.__odom_is_loging = True
    
    def end_odom_loging(self):
        self.__odom_is_loging = False

    @property
    def vel_log(self):
        return self.__tgt_vel_log, self.__odom_vel_log