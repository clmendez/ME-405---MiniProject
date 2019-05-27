# -*- coding: utf-8 -*-
#
## @privatesection - Stuff in this file doesn't need to be Doxygen-ed
#
#  @author jr

import pyb
import micropython
import gc

import cotask
import task_share
import print_task

import controller
import encoder
import motor
import time
import utime
import filter1
import acc

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
micropython.alloc_emergency_exception_buf (100)

pinC0 = pyb.Pin (pyb.Pin.board.PC0, pyb.Pin.ANALOG)
queue = task_share.Queue('f', 1000)
adc = pyb.ADC(pinC0)
pinC1 = pyb.Pin (pyb.Pin.board.PC1, pyb.Pin.OUT_PP)


def clear():
    print("\x1B\x5B2J", end="")
    print("\x1B\x5BH", end="")

def main():
    aye = pyb.I2C(1, pyb.I2C.MASTER)
    test = acc.Acc (aye, 107)
    filter0 = filter1.Filter(test, .98)
    while True:
        x_theta = filter0.updated_x()
        y_theta = filter0.updated_y()
        print("X acceleration raw: ", test.get_ax_int())
        print("Y acceletation raw: ", test.get_ay_int())
        print("Z acceletation raw: ", test.get_az_int())
        print("X acceleration: " + str(test.get_ax()) + " g")
        print("Y acceletation: " + str(test.get_ay()) + " g")
        print("Z acceletation: " + str(test.get_az()) + " g")
        print("updated x: " + str(x_theta))
        print("updated y: " + str(y_theta))
        utime.sleep_ms(500)
        
        clear()
        
        

# =============================================================================

if __name__ == "__main__":
    
    main()
    


    
    
    
