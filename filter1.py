# -*- coding: utf-8 -*-
"""
Created on Mon May 27 15:34:58 2019

@author: claud
"""

import math
import utime
  # Complimentary Filter
    
#bus = smbus.SMBus(0)  # or bus = smbus.SMBus(1) for Revision 2 boards

#bus.write_byte_data(address, power_mgmt_1, 0)

class Filter:
    
    def __init__(self, acc, K):
    
        self.gyro_scale = 15000
        self.tau = 0.075
        self.a = 0
        
    
        self.acc = acc
        self.gyro_x = (acc.get_gx_int() / self.gyro_scale)
        self.acc_y = acc.get_ay_int()
        self.acc_z = acc.get_az_int()
        self.gyro_offset_x = self.gyro_x
        self.comp_result = self.get_x_rotation(self.acc_y, self.acc_z)
        self.old_time = utime.ticks_ms()
        
    def get_x_rotation(self, y, z):
        radians = math.atan2(y, z)
        return math.degrees(radians)
    
    def updated_x(self):
        # get scaled gyro measurment
        gyro_scaled = (self.acc.get_gx_int() / self.gyro_scale) - self.gyro_offset_x 
        
        # get time difference for gyro integrator
        self.new_time = utime.ticks_ms()
        self.time_diff = self.new_time - self.old_time
        self.old_time = self.new_time
        
        gyro_delta = (gyro_scaled * self.time_diff)
        
        # get accelerometer rotation
        rotation = self.get_x_rotation(self.acc.get_ay_int(), self.acc.get_az_int())
        
        self.comp_result = 0.99*(self.comp_result + gyro_delta) + (.01)*rotation

        return self.comp_result
   
    

    
        
