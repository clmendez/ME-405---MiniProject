

## @file controller.py
#  Brief doc for encoder.py
#
#  Detailed doc for encoder.py 
#
#  @author Anthony Fortner, Claudia Mendez
#
#  @copyright License Info
#
#  @date April 22, 2019
#

import pyb
import utime
## 
#
#  A controller driver object to compute proportional error and gain control system
#
#
#  @author Anthony Fortner, Claudia Mendez
#  @date April 22, 2019

class Controller:
    
    ##  Constructor for controller driver
    #    
    #   @param kp control gain 
    #   @param setpoint desired location of the motor

    def __init__(self, kp, kd, ki, setpoint):

        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._setpoint= setpoint
        self._time_list = []
        self._pos_list = []
        self._start_time = int(utime.ticks_ms())
        self._error = 0
        self._time = utime.ticks_ms()
        self._integral = 0
        
          
    ## Computes the actuation signal to be sent to the motor driver
    #    
    #   @param position current position of the motor driver
    #   @return actuation signal 
        
    def calculate(self, position):
        
        # compute error and output
        self._error_old = self._error
        self._error = self._setpoint - position
        
        self._time_old = self._time
        self._time = utime.ticks_ms()
        
        
        self._error_diff = self._error - self._error_old
        self._time_diff = self._time - self._time_old
        self._error_derivative = self._error_diff / self._time_diff
        self._integral = self._integral + (self._error * self._time_diff)
        self._output = self._kp * self._error + self._kd * self._error_derivative + self._ki * self._integral
        
        print("p: " + str(self._error*self._kp) + " i: " + str(self._integral*self._ki) + " d: " + str(self._error_derivative*self._kd), end=" ")
        
        
        # saturation
        if (self._output > 100) :
            self._output = 100
        elif (self._output < -100) :
            self._output = -100

        #self._time_list.append(int(utime.ticks_ms()) - self._start_time)
        #self._pos_list.append(position)

        #print("calculate result: " + str(self._output))

        return self._output
        
    ## Sets the current gain value to a new value that is desired.
    #    
    #   @param setting_gain new value to set the gain value to
    
    def set_gain(self, setting_gain):
        self._kp = setting_gain

    ## Sets the current setpoint value to a new value that is desired. 
    #    
    #   @param setting_setpoint new value to set the setpoint value to       
        
    def set_setpoint (self, setting_setpoint):
        self._setpoint = setting_setpoint
  
    ## Prints the values that are in the _time_list and _pos_list variables. 
    #         
              
    def print_results (self):

        for i in range(len(self._time_list)) :
            print(str(self._time_list[i]) + ", " + str(self._pos_list[i]))


    ## Clears the values that are in the _time_list and _pos_list variables. 
    #         

    def clear_list (self):

        self._time_list = []
        self._pos_list = []
        self._start_time = int(utime.ticks_ms())




       
