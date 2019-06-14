# -*- coding: utf-8 -*-
"""
Created on Mon May 27 17:48:42 2019

@author: claud
"""

import pyb


repl = pyb.USB_VCP()
uart = pyb.UART(3, 9600)

while True:
    if uart.any():
        ch = uart.read()
        repl.write(ch)
    if repl.any():
        ch = repl.read()
        uart.write(ch)
        
