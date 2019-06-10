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
import sensor


# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
micropython.alloc_emergency_exception_buf (100)

pinC0 = pyb.Pin (pyb.Pin.board.PC0, pyb.Pin.ANALOG)
queue = task_share.Queue('f', 1000)
adc = pyb.ADC(pinC0)
pinC1 = pyb.Pin (pyb.Pin.board.PC1, pyb.Pin.OUT_PP)


# Task constants
BALANCE = 0
FORWARD = 1
BACKWARD = 2
TURNLEFT = 3
TURNRIGHT = 4
THETA_OFFSET = 0.5

# Global shared variable
FRONT_SENSOR = 0
BACK_SENSOR = 0
LEFT_SENSOR = 0
RIGHT_SENSOR = 0

BLUETOOTH_FORWARD = 0
BLUETOOTH_BACKWARD = 0
BLUETOOTH_TURNLEFT = 0
BLUETOOTH_TURNRIGHT = 0

BALANCE_SET_POINT = 0
FRONT_SET_POINT = 5
TURNING_SET_POINT = 5

def task_motor ():
    ''' Function which runs for Task 1, and controls how the motors will rotate.  '''
    
    control = controller.Controller(6, 0, 0, BALANCE_SET_POINT)
    
    motor1 = motor.MotorDriver(pyb.Pin.board.PB4,pyb.Pin.board.PB5, pyb.Pin.board.PA10, pyb.Timer(3))
    encoder1 = encoder.Encoder(pyb.Pin.board.PB7, pyb.Pin.board.PB6, pyb.Timer(4))
    
    motor2 = motor.MotorDriver(pyb.Pin.board.PA1,pyb.Pin.board.PA0, pyb.Pin.board.PC1, pyb.Timer(2))
    encoder2 = encoder.Encoder(pyb.Pin.board.PC7, pyb.Pin.board.PC6, pyb.Timer(8))

    aye = pyb.I2C(1, pyb.I2C.MASTER)
    accelo = acc.Acc (aye, 107)
    filter0 = filter1.Filter(accelo, .98)
    
    state = BALANCE
    
    while True:
        x_theta = (filter0.updated_x() + THETA_OFFSET)
        pwm = control.calculate(x_theta)
        clear()
        
        if state == BALANCE :
            '''Include what will make this robot balance, accelo info and filters, motor duty cycle'''
            control.set_setpoint(BALANCE_SET_POINT)
            motor1.set_duty_cycle(pwm)
            motor2.set_duty_cycle(pwm)
            
            if (BACK_SENSOR == 1 or BLUETOOTH_FORWARD == 1) :
                ''' go forward'''
                state = FORWARD
            elif (FRONT_SENSOR == 1 or BLUETOOTH_BACKWARD == 1) :
                ''' go backwards'''
        
                state = BACKWARD
            elif (RIGHT_SENSOR == 1 or BLUETOOTH_TURNLEFT == 1) :
                ''' turn left'''

                state = TURNLEFT
            elif (LEFT_SENSOR == 1 or BLUETOOTH_TURNRIGHT == 1) :
                ''' Turn right'''
                state = TURNRIGHT
                
        elif state == FORWARD :
            ''' Include what will make this robot move forward'''
            control.set_setpoint(FRONT_SET_POINT)
            motor1.set_duty_cycle(pwm)
            motor2.set_duty_cycle(pwm)
            
            if (FRONT_SENSOR == 1 or BLUETOOTH_BACKWARD == 1) :
                ''' go backwards'''
                state = BACKWARD
                
            elif (RIGHT_SENSOR == 1 or BLUETOOTH_TURNLEFT == 1):
                ''' turn left'''
                state = TURNLEFT
                
            elif (LEFT_SENSOR == 1 or BLUETOOTH_TURNRIGHT == 1):
                ''' Turn right'''
                state = TURNRIGHT
                
            elif (BACK_SENSOR == 0):
                state = BALANCE
                
        elif state == BACKWARD :
            ''' Include what will make this robot move backwards'''
            control.set_setpoint(FRONT_SET_POINT)
            motor1.set_duty_cycle(pwm)
            motor2.set_duty_cycle(pwm)
            
            if (BACK_SENSOR == 1 or BLUETOOTH_FORWARD == 1):
                ''' go forward'''
                state = FORWARD
                
            elif (RIGHT_SENSOR == 1 or BLUETOOTH_TURNLEFT == 1):
                ''' turn left'''
                state = TURNLEFT
                
            elif (LEFT_SENSOR == 1 or BLUETOOTH_TURNRIGHT == 1):
                ''' Turn right'''
                state = TURNRIGHT
                
            elif (FRONT_SENSOR == 0):
                state = BALANCE
            
        elif state == TURNLEFT :
            ''' Include what will make this robot turn left'''
            control.set_setpoint(FRONT_SET_POINT)
            motor1.set_duty_cycle(pwm)
            motor2.set_duty_cycle(pwm)

            if (BACK_SENSOR == 1 or BLUETOOTH_FORWARD == 1):
                ''' go forward'''
                state = FORWARD
                
            elif (FRONT_SENSOR == 1 or BLUETOOTH_BACKWARD == 1):
                ''' go backwards'''
                state = BACKWARD
                
            elif (LEFT_SENSOR == 1 or BLUETOOTH_TURNRIGHT == 1):
                ''' Turn right'''
                state = TURNRIGHT
                
            elif (RIGHT_SENSOR == 0):
                state = BALANCE
                    
        elif state == TURNRIGHT :
            ''' Include what will make this robot turn right'''
            control.set_setpoint(FRONT_SET_POINT)
            motor1.set_duty_cycle(pwm)
            motor2.set_duty_cycle(pwm)

            if (BACK_SENSOR == 1 or BLUETOOTH_FORWARD == 1) :
                ''' go forward'''
                state = FORWARD
                
            elif (FRONT_SENSOR == 1 or BLUETOOTH_BACKWARD == 1) :
                ''' go backwards'''               
                state = BACKWARD
                
            elif (RIGHT_SENSOR == 1 or BLUETOOTH_TURNLEFT == 1) :
                ''' turn left'''
                state = TURNLEFT
                
            elif (LEFT_SENSOR == 0):
                state = BALANCE
                        
        yield (state)

GATHER = 0
SEND = 1

def task_front_sensor ():
    ''' Function which runs for Task 2, which measures the distance noise is from the robot.  '''
    global FRONT_SENSOR
    state = GATHER
    sensor1 = sensor.Sensor(pyb.Pin.board.PA8, pyb.Pin.board.PA9)

    limit = 3000
    
    while True:
        
        if state == GATHER :
            ''' measures the distance robot is from objects based on noise, 5 sensors'''
            echo1 = sensor1.measure_distance()
    
            if (echo1 < limit):
                FRONT_SENSOR = 1
                state = SEND
                
        elif state == SEND :
            ''' sends single to motor task to move backward'''
            echo1 = sensor1.measure_distance()
            if (echo1 > limit):
                FRONT_SENSOR = 0
                state = GATHER
            
        yield (state)
 
def task_back_sensor ():
    ''' Function which runs for Task 2, which measures the distance noise is from the robot.  '''
    global BACK_SENSOR
    state = GATHER
    sensor1 = sensor.Sensor(pyb.Pin.board.PA8, pyb.Pin.board.PA9)

    limit = 3000
    
    while True:
        
        if state == GATHER :
            ''' measures the distance robot is from objects based on noise, 5 sensors'''
            echo1 = sensor1.measure_distance()
    
            if (echo1 < limit):
                BACK_SENSOR = 1
                state = SEND
                
        elif state == SEND :
            ''' sends single to motor task to move backward'''
            echo1 = sensor1.measure_distance()
            if (echo1 > limit):
                BACK_SENSOR = 0
                state = GATHER
            
        yield (state)
        
        
def task_left_sensor ():
    ''' Function which runs for Task 2, which measures the distance noise is from the robot.  '''
    global LEFT_SENSOR
    state = GATHER
    sensor1 = sensor.Sensor(pyb.Pin.board.PA8, pyb.Pin.board.PA9)

    limit = 3000
    
    while True:
        
        if state == GATHER :
            ''' measures the distance robot is from objects based on noise, 5 sensors'''
            echo1 = sensor1.measure_distance()
    
            if (echo1 < limit):
                LEFT_SENSOR = 1
                state = SEND
                
        elif state == SEND :
            ''' sends single to motor task to move backward'''
            echo1 = sensor1.measure_distance()
            if (echo1 > limit):
                LEFT_SENSOR = 0
                state = GATHER
            
        yield (state)
 
def task_right_sensor ():
    ''' Function which runs for Task 2, which measures the distance noise is from the robot.  '''
    global RIGHT_SENSOR
    state = GATHER
    sensor1 = sensor.Sensor(pyb.Pin.board.PA8, pyb.Pin.board.PA9)

    limit =35000
    
    while True:
        
        if state == GATHER :
            ''' measures the distance robot is from objects based on noise, 5 sensors'''
            echo1 = sensor1.measure_distance()
    
            if (echo1 < limit):
                RIGHT_SENSOR = 1
                state = SEND
                
        elif state == SEND :
            ''' sends single to motor task to move backward'''
            echo1 = sensor1.measure_distance()
            if (echo1 > limit):
                RIGHT_SENSOR = 0
                state = GATHER
            
        yield (state)

FORWARD = 1
BACKWARD = 2
TURNLEFT = 3
TURNRIGHT = 4

def task_bluetooth ():
    ''' Function which runs for Task 2, gives the robot commands based on user input.  '''
    global BLUETOOTH_FORWARD
    global BLUETOOTH_BACKWARD 
    global BLUETOOTH_TURNLEFT
    global BLUETOOTH_TURNRIGHT 
    
    #BlueTooth Set-up

    uart = pyb.UART(2,9600)
    uart.init(9600, bits=8, stop=1, parity=None)
    pyb.repl_uart(uart)
    
    state = GATHER
    
    while True:
        
        if state == GATHER:
            ''' waiting for signal to appear '''
            if uart.any() == 'forward':
                state = FORWARD
            elif uart.any() == 'backward':
                state = BACKWARD
            elif uart.any() == 'turnleft':
                state = TURNLEFT
            elif uart.any() == 'turnright':
                state = TURNRIGHT
                
        elif state == FORWARD:
            BLUETOOTH_FORWARD = 1
            if uart.any() == 'forward':
                state = FORWARD
            elif uart.any() == 'backward':
                state = BACKWARD
            elif uart.any() == 'turnleft':
                state = TURNLEFT
            elif uart.any() == 'turnright':
                state = TURNRIGHT
            else:
                BLUETOOTH_FORWARD = 0
                state = GATHER
                
        elif state == BACKWARD:
            BLUETOOTH_BACKWARD = 1
            if uart.any() == 'forward':
                state = FORWARD
            elif uart.any() == 'backward':
                state = BACKWARD
            elif uart.any() == 'turnleft':
                state = TURNLEFT
            elif uart.any() == 'turnright':
                state = TURNRIGHT
            else:
                BLUETOOTH_BACKWARD = 0
                state = GATHER
                
        elif state == TURNLEFT:
            BLUETOOTH_TURNLEFT = 1
            if uart.any() == 'forward':
                state = FORWARD
            elif uart.any() == 'backward':
                state = BACKWARD
            elif uart.any() == 'turnleft':
                state = TURNLEFT
            elif uart.any() == 'turnright':
                state = TURNRIGHT
            else:
                BLUETOOTH_TURNLEFT = 0
                state = GATHER
                
        elif state == TURNRIGHT:
            BLUETOOTH_TURNRIGHT = 1
            if uart.any() == 'forward':
                state = FORWARD
            elif uart.any() == 'backward':
                state = BACKWARD
            elif uart.any() == 'turnleft':
                state = TURNLEFT
            elif uart.any() == 'turnright':
                state = TURNRIGHT
            else:
                BLUETOOTH_TURNRIGHT = 0
                state = GATHER
            
                
        yield (state)
                
                

def clear():
    print("\x1B\x5B2J", end="")
    print("\x1B\x5BH", end="")

def main():
    
    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    motor_task = cotask.Task (task_motor, name = 'motor_task', priority = 1,
                            period = 1, profile = True, trace = False)
    
 #   front_sensor_task = cotask.Task (task_front_sensor, name = 'front_sensor_task', priority = 2,
 #                           period = 25, profile = True, trace = False)
    
    #back_sensor_task = cotask.Task (task_back_sensor, name = 'back_sensor_task', priority = 3,
    #                        period = 25, profile = True, trace = False)
    
    #right_sensor_task = cotask.Task (task_right_sensor, name = 'right_sensor_task', priority = 4,
    #                        period = 25, profile = True, trace = False)
    
    #left_sensor_task = cotask.Task (task_left_sensor, name = 'left_sensor_task', priority = 5,
    #                        period = 25, profile = True, trace = False)
    
    #bluetooth_task = cotask.Task (task_bluetooth, name = 'bluetooth_task', priority = 6,
    #                        period = 25, profile = True, trace = False)
    
    cotask.task_list.append (motor_task)
  #  cotask.task_list.append (front_sensor_task)
    #cotask.task_list.append (back_sensor_task)
    #cotask.task_list.append (right_sensor_task)
    #cotask.task_list.append (left_sensor_task)
    #cotask.task_list.append (bluetooth_task)
    
    # A task which prints characters from a queue has automatically been
    # created in print_task.py; it is accessed by print_task.put_bytes()

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()

    # Run the scheduler with the chosen scheduling algorithm. Quit if any 
    # character is sent through the serial por
    vcp = pyb.USB_VCP ()
    while not vcp.any ():
        cotask.task_list.pri_sched ()

    # Empty the comm port buffer of the character(s) just pressed
    vcp.read ()

    # Print a table of task data and a table of shared information data
    print ('\n' + str (cotask.task_list) + '\n')
    print (task_share.show_all ())
   # print (control_task.get_trace ())
    print ('\r\n')
    
    '''  aye = pyb.I2C(1, pyb.I2C.MASTER)
    test = acc.Acc (aye, 107)
    filter0 = filter1.Filter(test, .98)
    sensor0 = sensor.Sensor(pyb.Pin.board.PA8,pyb.Pin.board.PA9)
    
    while True:
        x_theta = filter0.updated_x()
        y_theta = filter0.updated_y()
        echo = sensor0.measure_distance()
     #   print("X acceleration raw: ", test.get_ax_int())
      #  print("Y acceletation raw: ", test.get_ay_int())
       # print("Z acceletation raw: ", test.get_az_int())
        #print("X acceleration: " + str(test.get_ax()) + " g")
        #print("Y acceletation: " + str(test.get_ay()) + " g")
        #print("Z acceletation: " + str(test.get_az()) + " g")
        #print("updated x: " + str(x_theta))
        #print("updated y: " + str(y_theta))
     #   print("echo: " + str(echo))
        utime.sleep_ms(500)
        
        clear()
        '''
        

# =============================================================================

if __name__ == "__main__":
    
    main()
    


    
    
    
