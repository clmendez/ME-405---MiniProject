## @file sensor.py
#  Brief doc for sensor.py
#
#  Detailed doc for sensor.py 
#
#  @author Anthony Fortner, Claudia Mendez
#
#  @copyright License Info
#
#  @date June 4, 2019

import pyb
import utime
import machine


class Sensor:
    
    ## Constructor for Sensor driver
    #
    #  Creates a Sensor driver by initializing 
    #    
    #  @param pin1 pyb.Pin.board.XXX pin object that connects 
    #  @param pin2 pyb.Pin.board.XXX pin object that connects
    
    def __init__(self, pin1, pin2):
        
        self.echo = pyb.Pin (pin1, pyb.Pin.IN)
        
        self.trigger = pyb.Pin (pin2, pyb.Pin.OUT)
        
        
    ## This method gets the distance from noise that the sensor detects.    
    #  @return distance of noise that the sensor detects
    
    def measure_distance(self):
        self.trigger.high()
        utime.sleep_us(10)
        self.trigger.low()
        pulse_time = machine.time_pulse_us(self.echo,1, 50000)
        return pulse_time
        
        
        

        