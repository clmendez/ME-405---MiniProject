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
    #  Creates a Sensor driver by initializing the pins required to interact with the ultrasonic sensor. 
    #    
    #  @param pin1 pyb.Pin.board.XXX pin object that connect to the Pyboard
    #  @param pin2 pyb.Pin.board.XXX pin object that connect to the Pyboard
    
    def __init__(self, pin1, pin2):
        
        ## Echo is the pulse that we recieve from the ultrasonic sensor 
        self.echo = pyb.Pin (pin1, pyb.Pin.IN)
        
        ## Trigger is a pulse that we send to the ultrasonic sensor 
        self.trigger = pyb.Pin (pin2, pyb.Pin.OUT)
        
        
    ## This method gets the distance from noise that the sensor detects.    
    #  @return distance of noise that the sensor detects
    
    def measure_distance(self):
        self.trigger.high()
        utime.sleep_us(10)
        self.trigger.low()
        pulse_time = machine.time_pulse_us(self.echo,1, 50000)
        print(pulse_time)
        return pulse_time
        
        
        

        