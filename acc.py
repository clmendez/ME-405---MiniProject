# -*- coding: utf-8 -*-
"""
Created on Mon May 20 20:30:58 2019

@author: claud
"""

import micropython
import struct

import smbus
import math
import time



TEST_PAGE  = micropython.const (0x00)
RAM_ACCESS = micropython.const (0x01)
SENSOR_SYNC_TIME = micropython.const (0x04)
SENSOR_SYNC_EN = micropython.const (0x05)	
FIFO_CTRL1  =	micropython.const (0x06)		
FIFO_CTRL2 = micropython.const (0x07)		
FIFO_CTRL3  = micropython.const (0x08)
FIFO_CTRL4  = micropython.const (0X09)
FIFO_CTRL5  = micropython.const (0X0A)
ORIENT_CFG_G  = micropython.const (	0X0B)
REFERENCE_G  = micropython.const (0X0C)
INT1_CTRL  	= micropython.const (0X0D)
INT2_CTRL  	= micropython.const (0X0E)
WHO_AM_I  = micropython.const (0X0F)
CTRL1_XL  	= micropython.const (0X10)
CTRL2_G  	= micropython.const (0X11)
CTRL3_C  	= micropython.const (0X12)
CTRL4_C  	= micropython.const (0X13)
GYRO_CTRL5_C  = micropython.const (0X14)
#define LSM6DS3_ACC_GYRO_CTRL6_G  			0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  			0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  			0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  			0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  			0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  	0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  		0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  			0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  			0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  			0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L  			0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H  			0X21
OUTX_L_G  = micropython.const (	0X22)
OUTX_H_G  = micropython.const (	0X23)
OUTY_L_G  = micropython.const (	0X24)
OUTY_H_G  = micropython.const (	0X25)
OUTZ_L_G  = micropython.const (	0X26)
OUTZ_H_G  = micropython.const (	0X27)

OUTX_L_XL = micropython.const (0X28)
OUTX_H_XL = micropython.const (	0X29)
OUTY_L_XL = micropython.const (0X2A)
OUTY_H_XL  = micropython.const (0X2B)
OUTZ_L_XL  = micropython.const (0X2C)
OUTZ_H_XL  = micropython.const (0X2D)
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  		0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  		0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  		0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  		0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  		0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  		0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  		0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  		0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  		0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  		0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  		0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  		0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  			0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  			0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  			0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  			0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  		0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  		0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  		0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  		0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  		0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  		0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  		0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC  			0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1  			0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  			0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  			0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  			0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  			0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  			0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  			0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  			0X5F

#define LSM6DS3_ACC_GYRO_ADDR0_TO_RW_RAM         0x62
#define LSM6DS3_ACC_GYRO_ADDR1_TO_RW_RAM         0x63
#define LSM6DS3_ACC_GYRO_DATA_TO_WR_RAM          0x64
#define LSM6DS3_ACC_GYRO_DATA_RD_FROM_RAM        0x65

#define LSM6DS3_ACC_GYRO_RAM_SIZE                4096


#define LSM6DS3_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DS3_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DS3_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DS3_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DS3_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DS3_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DS3_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DS3_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DS3_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DS3_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DS3_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DS3_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DS3_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_IIR              0x10
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_ACOS             0x11
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_WTIME            0x12
#define LSM6DS3_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DS3_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DS3_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DS3_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DS3_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DS3_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DS3_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DS3_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DS3_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DS3_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DS3_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DS3_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DS3_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DS3_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_H                   0x32

class Acc:
    
    def __init__(self, i2c, address, accel_range = 0):
        ## The I2C driver which was created by the code which called this
        self.i2c = i2c

        ## The I2C address at which the accelerometer is located
        self.addr = address

        # Request the WHO_AM_I device ID byte from the accelerometer
        self._dev_id = ord (i2c.mem_read (1, address, WHO_AM_I))
        # The WHO_AM_I codes from MMA8451Q's and MMA8452Q's are recognized
        if self._dev_id == 0x69:
            self._works = True
        else:
            self._works = False
            raise ValueError ('Unknown accelerometer device ID ' 
                + str (self._dev_id) + ' at I2C address ' + str(address))
            
        self.i2c.mem_write(0x47, address,CTRL1_XL)
        self.i2c.mem_write(0x80, address,CTRL4_C )
        self.i2c.mem_write(0x4C, address,CTRL2_G )
            
        ## Accelerations
            
    def get_ax_int (self):
        """ Get the X acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured X acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTX_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]
    
    def get_ay_int (self):
        """ Get the Y acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Y acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTY_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]


    def get_az_int (self):
        """ Get the Z acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Z acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTZ_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]


    def get_ax (self):
        """ Get the X acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory.
        @return The measured X acceleration in g's """

        return self.get_ax_int()/2035


    def get_ay (self):
        """ Get the Y acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Y acceleration in g's """

        return self.get_ay_int()/2030


    def get_az (self):
        """ Get the Z acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Z acceleration in g's """

        return self.get_az_int()/2040
    
    


    def get_accels (self):
        """ Get all three accelerations from the MMA845x accelerometer. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return A tuple containing the X, Y, and Z accelerations in g's """

        return (self.get_ax (), self.get_ay (), self.get_az ())
        
    
    ## Gyro
    
    def get_gx_int (self):
        """ Get the X angular acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured X acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTX_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]
    
    def get_gy_int (self):
        """ Get the Y acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Y acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTY_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]


    def get_gz_int (self):
        """ Get the Z acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Z acceleration in A/D conversion bits """

        byte_array = self.i2c.mem_read(2, self.addr, OUTZ_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]


    def get_gx (self):
        """ Get the X acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory.
        @return The measured X acceleration in g's """

        return self.get_ax_int()/2035


    def get_gy (self):
        """ Get the Y acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Y acceleration in g's """

        return self.get_ay_int()/2030


    def get_gz (self):
        """ Get the Z acceleration from the accelerometer in g's, assuming
        that the accelerometer was correctly calibrated at the factory. The
        measurement is adjusted for the range (2g, 4g, or 8g) setting.
        @return The measured Z acceleration in g's """

        return self.get_az_int()/2040
    
    
    
    
    
    def get_y_rotation(x,y,z):
        radians = math.atan2(x, dist(y,z))
        return -math.degrees(radians)

    def get_x_rotation(x,y,z):
        radians = math.atan2(y, dist(x,z))
        return math.degrees(radians)
 
bus = smbus.SMBus(0)  # or bus = smbus.SMBus(1) for Revision 2 boards

bus.write_byte_data(address, power_mgmt_1, 0)
52
 
53
now = time.time()
54
 
55
K = 0.98
56
K1 = 1 - K
57
 
58
time_diff = 0.01
59
 
60
(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
61
 
62
last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
63
last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
64
 
65
gyro_offset_x = gyro_scaled_x
66
gyro_offset_y = gyro_scaled_y
67
 
68
gyro_total_x = (last_x) - gyro_offset_x
69
gyro_total_y = (last_y) - gyro_offset_y
70
 
71
print "{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - now, (last_x), gyro_total_x, (last_x), (last_y), gyro_total_y, (last_y))
72
 
73
for i in range(0, int(3.0 / time_diff)):
74
    time.sleep(time_diff - 0.005)
75
     
76
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
77
     
78
    gyro_scaled_x -= gyro_offset_x
79
    gyro_scaled_y -= gyro_offset_y
80
     
81
    gyro_x_delta = (gyro_scaled_x * time_diff)
82
    gyro_y_delta = (gyro_scaled_y * time_diff)
83
 
84
    gyro_total_x += gyro_x_delta
85
    gyro_total_y += gyro_y_delta
86
 
87
    rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
88
    rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
89
 
90
    last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
91
    last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)
92
     
93
    print "{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - now, (rotation_x), (gyro_total_x), (last_x), (rotation_y), (gyro_total_y), (last_y))
        
