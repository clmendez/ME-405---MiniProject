# -*- coding: utf-8 -*-
"""
Created on Mon May 27 16:57:52 2019

@author: claud
"""
import pyb
import utime
import machine


class Sensor:
    
    def __init__(self, pin1, pin2):
        self.echo = pyb.Pin (pin1, pyb.Pin.IN)
        self.trigger = pyb.Pin (pin2, pyb.Pin.OUT)
        
        
        
        
    def measure_distance(self):
        self.trigger.high()
        utime.sleep_us(10)
        self.trigger.low()
        pulse_time = machine.time_pulse_us(self.echo,1, 50000)
        return pulse_time
        
        
        

        