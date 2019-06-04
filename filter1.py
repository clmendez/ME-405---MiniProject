## @file filter1.py
#  Brief doc for filter1.py
#
#  Detailed doc for filter1.py 
#
#  @author Anthony Fortner, Claudia Mendez
#
#  @copyright License Info
#
#  @date June 4, 2019

import math
import time

## 
#
#  This class implements the complimentary filter used to determine the angle of the IMU.
#
#  @author Anthony Fortner, Claudia Mendez
#  @date June 4, 2019

class Filter:
    
    ## Constructor for Filter class 
    #
    #  Creates a Filter class by initializing 
    #    
    #   @param acc Acc object for communicating between the IMU and pyboard
    #   @param K 
    
    def __init__(self, acc, K):
        self._acc = acc
        self._now = time.time()
        self._K = K
        self._K1 = 1 - self.K
        self._time_diff = 0.01
        self._gyro_x = acc.get_gx_int()
        self._gyro_y = acc.get_gy_int()
        self._gyro_z = acc.get_gz_int()
        self._acc_x = acc.get_ax_int()
        self._acc_y = acc.get_ay_int()
        self._acc_z = acc.get_az_int()
        
        ##  The x-axis offset is the value of the gyroscope reading when it's not moving and is taken from the very first reading in the x-direction.
        self.gyro_offset_x = self.gyro_x
        
        ##  The y-axis offset is the value of the gyroscope reading when it's not moving and is taken from the very first reading in the y-direction.
        self.gyro_offset_y = self.gyro_y
        
        ##  The z-axis offset is the value of the gyroscope reading when it's not moving and is taken from the very first reading in the z-direction.
        self.gyro_offset_z = self.gyro_z
        self._last_x = self.get_x_rotation(self.acc_x, self.acc_y, self.acc_z)
        self._last_y = self.get_y_rotation(self.acc_x, self.acc_y, self.acc_z)
        self._gyro_total_x = self.last_x - self.gyro_offset_x
        self._gyro_total_y = self.last_y - self.gyro_offset_y
        
        
    ## This method returns the distance between two locations. 
    #  @return The distance between two locations. 
    
    def dist(self,a, b):
        return math.sqrt((a * a) + (b * b))

    ## This method 
    #  @return

    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)
    
    ## This method
    #  @return

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)
    
    ##  This method puts the IMU to sleep for a certain period of time. 
    
    def sleep (self):
        return self.time.sleep(self._time_diff - 0.005)
    
    
    ##  This method returns the current angle that the accelerometer is located in the X-axis
    #   @return Current angle of the accelerometer in the X-axis
    
    def updated_x(self):
        gyro_scaled = self.acc.get_gx_int() - self.gyro_offset_x 
        gyro_delta = gyro_scaled * self._time_diff
        self._gyro_total_x += gyro_delta
        rotation = self.get_x_rotation(self.acc.get_ax_int(), self.acc.get_ay_int(), self.acc.get_az_int())
        result  = self._K1 * ( self._last_x + gyro_delta) + (self._K * rotation)
        print("x rotation: " + str(result))
        return result

    ##  This method returns the current angle that the accelerometer is located in the Y-axis
    #   @return Current angle of the accelerometer in the Y-axis
    
    def updated_y(self):
        gyro_scaled = self.acc.get_gy_int() - self.gyro_offset_y
        gyro_delta = gyro_scaled * self._time_diff
        self._gyro_total_y += gyro_delta
        rotation = self.get_y_rotation(self.acc.get_ax_int(), self.acc.get_ay_int(), self.acc.get_az_int())
        result  = self._K1 * ( self.last_y + gyro_delta) + (self._K * rotation)
        return result
    
   
    

    
        
