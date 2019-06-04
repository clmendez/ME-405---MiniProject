## @file acc.py
#  Brief doc for acc.py
#
#  Detailed doc for acc.py 
#
#  @author Anthony Fortner, Claudia Mendez
#
#  @copyright License Info
#
#  @date June 4, 2019

import micropython
import struct

import smbus
import math
import time


## 
#
#  This class implements the IMU driver that measures the acceleration and gyroscope of the IMU. 
#
#  @author Anthony Fortner, Claudia Mendez
#  @date June 4, 2019



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
    
    ## Constructor for IMU driver
    #
    #  Creates a IMU driver by initializing 
    #    
    #   @param i2c I2C object for communicating between the IMU and pyboard
    #   @param address I2C address that the accelerometer is located at
    #   @param accel_range
    
    
    def __init__(self, i2c, address, accel_range = 0):
        self.i2c = i2c

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
            
        
    ## This method returns the X acceleration from the accelerometer in A/D bits
    #  @return The measured X acceleration in A/D conversion bits
            
    def get_ax_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTX_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]
    
    ## This method returns the Y acceleration from the accelerometer in A/D bits
    #  @return The measured Y acceleration in A/D conversion bits
    
    def get_ay_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTY_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]
    
    ## This method returns the Z acceleration from the accelerometer in A/D bits
    #  @return The measured Z acceleration in A/D conversion bits

    def get_az_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTZ_L_XL)        
        result = struct.unpack('<h', byte_array)
        return result[0]

    ## This method returns the X acceleration from the accelerometer in g's, assuming
    #  that the accelerometer was correctly calibrated at the factory
    #  @ return The measured X acceleration in g's
    
    def get_ax (self):
        return self.get_ax_int()/2035


    ## This method returns the Y acceleration from the accelerometer in g's, assuming
    #  that the accelerometer was correctly calibrated at the factory
    #  @ return The measured Y acceleration in g's
    
    def get_ay (self):
        return self.get_ay_int()/2030


    ## This method returns the Z acceleration from the accelerometer in g's, assuming
    #  that the accelerometer was correctly calibrated at the factory. 
    #  @ return The measured Z acceleration in g's

    def get_az (self):
        return self.get_az_int()/2040
    
    ## Gets the X angular accelertion from the accelerometer in A/D bits.
    #  @return The measured X angular acceleration in A/D conversion bits
    
    def get_gx_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTX_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]
    
    ## Gets the Y angular accelertion from the accelerometer in A/D bits.
    #  @return The measured Y angular acceleration in A/D conversion bits
    
    def get_gy_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTY_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]


    ## Gets the Z angular accelertion from the accelerometer in A/D bits.
    #  @return The measured Z angular acceleration in A/D conversion bits
    
    def get_gz_int (self):
        byte_array = self.i2c.mem_read(2, self.addr, OUTZ_L_G)        
        result = struct.unpack('<h', byte_array)
        return result[0]

    ## Gets the X angular accelertion from the accelerometer in g's, assuming that the 
    #  accelerometer was correctly calibrated at the factory.
    #  @return The measured X angular acceleration in g's
    
    def get_gx (self):
        return self.get_ax_int()/2035


    ## Gets the Y angular accelertion from the accelerometer in g's, assuming that the 
    #  accelerometer was correctly calibrated at the factory.
    #  @return The measured Y angular acceleration in g's

    def get_gy (self):
        return self.get_ay_int()/2030

    ## Gets the Z angular accelertion from the accelerometer in g's, assuming that the 
    #  accelerometer was correctly calibrated at the factory.
    #  @return The measured Z angular acceleration in g's
    
    def get_gz (self):
        return self.get_az_int()/2040
    
          

 
 
    
  