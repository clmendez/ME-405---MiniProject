## @file motor.py
#  Brief doc for motor.py
#
#  Detailed doc for motor.py 
#
#  @author Anthony Fortner, Claudia Mendez
#
#  @copyright License Info
#
#  @date June 4, 2019



import pyb


## 
#
#  This class implements a motor driver to control the direction and speed of a motor. 
#
#  @author Anthony Fortner, Claudia Mendez
#  @date June 4, 2019


class MotorDriver:
        
    ## Constructor for motor driver
    #
    #  Creates a motor driver by initializing GPIO
    #  pins and turning the motor off for safety.
    #    
    #   @param pin1 pyb.Pin.board.XXX pin object that connects to the forward control pin
    #   @param pin2 pyb.Pin.board.XXX pin object that connects to the reverse control pin
    #   @param pin3 pyb.Pin.board.XXX pin object that enables the motor
    #   @param timer pyb.Timer object with channels that can interface with pin1, and pin2
    def __init__ (self, pin1, pin2, pin3, timer):

        print ('Creating a motor driver')
        

        self._DEAD_ZONE = 10

        
        ## pinA Motor Controller forward control pin
        self._pinC = pyb.Pin (pin3, pyb.Pin.OUT_PP)
        self._pinC.high ()
        
        ## pinA Motor Controller forward control pin
        self._pinA = pyb.Pin (pin1, pyb.Pin.OUT_PP)
    
        ## pinB Motor Controller reverse control pin
        self._pinB = pyb.Pin (pin2, pyb.Pin.OUT_PP)
        
        self._tim = timer
        self._tim.init(freq=20000)
        
        self._ch1 = self._tim.channel (1, pyb.Timer.PWM, pin=self._pinA)
        self._ch2 = self._tim.channel (2, pyb.Timer.PWM, pin=self._pinB)
        
        self._ch1.pulse_width_percent (0)
        self._ch2.pulse_width_percent (0)
        
    ## This method sets the duty cycle to be sent
    #  to the motor to the given level. Positive values
    #  cause torque in one direction, negative values
    #  in the opposite direction.
    #  @param level A signed integer holding the duty
    #  cycle of the voltage sent to the motor. Range -100 to 100. 
    def set_duty_cycle (self, level):
        #print ('Setting duty cycle to ' + str (level))
        
        if (level + self._DEAD_ZONE) > 100 :
            level -= self._DEAD_ZONE 
        elif (level - self._DEAD_ZONE) < -100 :
            level += self._DEAD_ZONE
        
        if (level > 0) :
            self._ch2.pulse_width_percent (0)
            print("pwm: " + str(level + self._DEAD_ZONE), end=" ")
            self._ch1.pulse_width_percent (level + self._DEAD_ZONE)
        else:
            self._ch1.pulse_width_percent (0)
            print("pwm: " + str((-level) + self._DEAD_ZONE), end=" ")
            self._ch2.pulse_width_percent ((-level) + self._DEAD_ZONE)