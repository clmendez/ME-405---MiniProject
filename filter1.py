# -*- coding: utf-8 -*-
"""
Created on Mon May 27 15:34:58 2019

@author: claud
"""

import math
import time
  # Complimentary Filter
    
#bus = smbus.SMBus(0)  # or bus = smbus.SMBus(1) for Revision 2 boards

#bus.write_byte_data(address, power_mgmt_1, 0)

class Filter:
    
    def __init__(self, acc, K):
        self.acc = acc
        self.now = time.time()
        self.K = K
        self.K1 = 1 - self.K
        self.time_diff = 0.01
        self.gyro_x = acc.get_gx()
        self.gyro_y = acc.get_gy()
        self.gyro_z = acc.get_gz()
        self.acc_x = acc.get_ax()
        self.acc_y = acc.get_ay()
        self.acc_z = acc.get_az()
        self.gyro_offset_x = self.gyro_x
        self.gyro_offset_y = self.gyro_y
        self.gyro_offset_z = self.gyro_z
        self.last_x = self.get_x_rotation(self.acc_x, self.acc_y, self.acc_z)
        self.last_y = self.get_y_rotation(self.acc_x, self.acc_y, self.acc_z)
        self.gyro_total_x = self.last_x - self.gyro_offset_x
        self.gyro_total_y = self.last_y - self.gyro_offset_y
        
        
    def dist(self,a, b):
        return math.sqrt((a * a) + (b * b))


    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)
    
    
    def sleep (self):
        return self.time.sleep(self.time_diff - 0.005)
    
    def all_measurements (self):
        return print("Acc x: " + str(self.acc.get_ax()) +
                     "Acc y: " + str(self.acc.get_ay()) +
                     "Acc z: " + str(self.acc.get_az()) +
                     "Gyro x: " + str(self.acc.get_gx()) +
                     "Gyro y: " + str(self.acc.get_gy()) +
                     "Gyro z: " + str(self.acc.get_gz()))
        
        
    def updated_x(self):
        gyro_scaled = self.acc.get_gx() - self.gyro_offset_x 
        gyro_delta = gyro_scaled * self.time_diff
        self.gyro_total_x += gyro_delta
        rotation = self.get_x_rotation(self.acc.get_ax(), self.acc.get_ay(), self.acc.get_az())
        result  = self.K * ( self.last_x - gyro_delta) + (self.K1 * rotation)
        return result

    def updated_y(self):
        gyro_scaled = self.acc.get_gy() - self.gyro_offset_y
        gyro_delta = gyro_scaled * self.time_diff
        self.gyro_total_y += gyro_delta
        rotation = self.get_y_rotation(self.acc.get_ax(), self.acc.get_ay(), self.acc.get_az())
        result  = self.K * ( self.last_y - gyro_delta) + (self.K1 * rotation)
        return result
    
   
    

    
        
