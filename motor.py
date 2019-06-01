import pyb

class MotorDriver:
    
    ## pinA10 enable pin, set high to enable Motor Controller
    
    
    ## Constructor for motor driver
    #
    #  Creates a motor driver by initializing GPIO
    #  pins and turning the motor off for safety.
    def __init__ (self, pin1, pin2, pin3, timer):

        print ('Creating a motor driver')
        
        ## pinB4 Motor Controller forward control pin
        self._pinC = pyb.Pin (pin3, pyb.Pin.OUT_PP)
        self._pinC.high ()
        
        
        self._pinA = pyb.Pin (pin1, pyb.Pin.OUT_PP)
    
        ## pinB5 Motor Controller reverse control pin
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
        
        if (level > 0) :
            self._ch2.pulse_width_percent (0)
            self._ch1.pulse_width_percent (level)
        else:
            self._ch1.pulse_width_percent (0)
            self._ch2.pulse_width_percent (-level)