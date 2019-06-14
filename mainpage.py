   ## @file mainpage.py
   #  Includes documentation for the entire project shown on the documentation landing page.
   #
   #  @author Anthony Fortner, Claudia Mendez
   #
   #  @mainpage
   #
   #  @section sec_intro Introduction
   #  Our driver is used to help in the creation of a two-wheeled robot that can balance on it's own. 
   #  The robot is open-loop unstable and stable in closed-loop. The robot is 
   #  also able to receive a remote input from a user to drive the robot.  
   #
   #  @section sec_Motor Motor Driver
   # 
   #  @subsection subsec_pur Purpose 
   #  The purpose of the motor driver is to control the direction and speed of a motor. 
   #
   #  @subsection subsec_use Usage
   #  To use the motor driver the following format is used:
   #
   #  motor.MotorDriver(pin1, pin2, pin3, timer)
   #  
   #  Please see motor.MotorDriver for more parameter specifications for the motor driver. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the driver worked properly we inputted two different speeds in the set_duty_cycle function
   #  of the driver to ensure that it changed speeds. We also tested to make sure that the motor would
   #  rotate in two different directions by inputting negative and positive values in the set_duty_cycle function. 
   #    
   #    
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. As for limitations ,the set_duty_cycle function
   #  of the motor can only be set in a range of -100 to 100. 
   #    
   #
   #  @section sec_Encoder Encoder Driver
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the encoder driver is to measure the rotational position of a motor. 
   #
   #  @subsection subsec_use Usage
   #  To use the encoder driver the following format is used:
   #
   #  encoder.Encoder(pin1, pin2, timer)
   #  
   #  Please see encoder.Encoder for more parameter specifications for the encoder driver. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the driver worked properly we rotated the 
   #  motor by hand in one direction to see if the position of the motor increased
   #  or decreased, which it did. We then moved the motor in the other direction 
   #  to test if the motor's position moved in the opposite direction, which it did. 
   #    
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. As for limitations our driver measurment is based on shortest movement on an integer ring mod 2^16\n
   #  This means that the encoder needs to be measured at a frequency equal to:
   #
   #  freq = (target Rev/min) * (encoder ticks/revolution) * 5.09*10^-7
   #
   #  @section sec_Controller Controller Driver
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the controller driver is to implement a proportional control loop. 
   #
   #  @subsection subsec_use Usage
   #  To use the controller driver the following format is used:
   #
   #  controller.Controller(kp,kd,ki, setpoint)
   #  
   #  Please see controller.Controller for more parameter specifications for the controller driver. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the driver worked properly we created a main function that 
   #  communicated with a host computer. The host comuputer commuicated over a
   #  serial interface that sets a variety of gain values. From those gain values,
   #  we checked to make sure a step response graph was produced with reasonable values.    
   #    
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. The integral portion of the control loop
   #  never clears, which could lead to boundless corrections if the system is not properly
   #  responding. 
   #
   #  @section sec_IMU IMU Driver
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the IMU driver is to measure the acceleration and gyroscope of the IMU. 
   #
   #  @subsection subsec_use Usage
   #  To use the IMU driver the following format is used:
   #
   #  acc.Acc(i2c, address, accel_range = 0)
   #  
   #  Please see acc.Acc for more parameter specifications for the IMU driver. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the driver worked properly we created a main function that 
   #  called created an acc class and used the function within that returned
   #  the measured acclerations in g's. We moved the IMU in each direction (x-axis and y-axis)
   #  to determine if they were being read correctly. 
   #      
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. As for limitations, the readings received from 
   #  the IMU could pick up noise which could result in the values being measured to be 
   #  incorrect. This issue with noise is reduced with the use of the complimentary filter. 
   # 
   #  @section sec_Filter Filter 
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the Filter class is to use the accelerations and gyroscopes measured by the IMU driver to determine the angle of the IMU. 
   #
   #  @subsection subsec_use Usage
   #  To use the filter class the following format is used:
   #
   #  filter1.Filter(acc)
   #  
   #  Please see filter1.Filter for more parameter specifications for the Filter class. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the filter class worked properly we created a main function that 
   #  created a filter class and used the function within the class that returned the measured angle of the IMU.
   #  We moved the IMU in each direction (x-axis and y-axis)to determine if they were being read correctly. 
   #      
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. As for limitations, the complimentary filter 
   #  is nonideal compared to the kalamin filter. 
   #  
   # 
   #  @section sec_Sensor Sensor 
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the Sensor class is to measure distances that the sensors detect, so that the robot can avoid them. 
   #
   #  @subsection subsec_use Usage
   #  To use the Sensor class the following format is used:
   #
   #  sensor.Sensor(pin1, pin2)
   #  
   #  Please see sensor.Sensor for more parameter specifications for the Sensor class. 
   #  
   #  @subsection subsec_test Testing 
   #  To test if the Sensor class worked properly we created a main function that 
   #  created a sesnor class and used the function within the class that returned the distance of an object that the sensor detects.
   #  We then made noises at a variety of distances to see if the sensor picked them up properly. 
   #      
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. As for limitations, the sensor can 
   #  pick up noise that isn't really there, which could result in the robot thinking
   #  it needs to move when it really doesn't. 
   #
   #  @section sec_main Main 
   #
   #  @subsection subsec_pur Purpose 
   #  The purpose of the main is to initialize all the tasks, which makes the robot balance. 
   #  It is bringing together all of the classes and seeing how they all work together. 
   #
   #  @subsection subsec_test Testing 
   #  To test the main we created tasks that made the classes. The three tasks that were
   #  made were a motor, sensor, and bluetooth task. We ran the tasks in a schedule to 
   #  see how the robot interacted.
   #  
   #  @subsection subsec_bug Bugs and Limitations 
   #  There were no known bugs in our code. Limitations are unknown. 
   #
   #  @section sec_link GitHub Link
   #  To get access to all the code listed above please see the following link:
   #
   #  https://github.com/clmendez/ME-405---MiniProject

