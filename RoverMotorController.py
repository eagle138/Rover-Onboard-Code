#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverMotorController.py
#
# PURPOSE: Interfaces with the MCP4725 12-bit DAC and MDC151-024031 Brushless
#          speed controller to provide control of the rover drive motors.
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for I2C communications
import smbus

# Imported to set the connection status of the motor controller
import RoverStatus

# Imported for RPi GPIO control
import RPi.GPIO as GPIO

# Imported for frequency measurement for motor speed
import time

# Imported for creation of brake thread
import multiprocessing

#******************************************************************************
#                              GLOBAL VARIABLES
#******************************************************************************

# I2C address of the MCP4725 DAC
DAC_ADDR = 0x62

# Command to write  an output voltage value to the MCP4725 DAC
CMD_WRITEDAC = 0x40

# Motor speed increment for each iteration of the adaptive braking
MOTOR_SPEED_INCREMENT = 0.07

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class MotorController():

    #--------------------------------------------------------------------------
    # MotorController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
    
        # Set up the variable to hold the brake process so that it can be 
        # started or terminated by different functions
        self.brakeProcess = None
    
        # Set up a variable to hold the current expected motor speed.
        self.motorSpeed = 0
    
        # Set the motor controller pins to RUN and ENABLE
        GPIO.output(RoverStatus.PIN_WHEEL_RUNSTOP, 0)
        GPIO.output(RoverStatus.PIN_WHEEL_COAST, 1)
    
        # Set up the I2C bus
        self.I2Cbus = smbus.SMBus(1)
        
        try:
        
            # Attempt to set the rover motor speed to zero through the DAC
            self.I2Cbus.write_i2c_block_data(DAC_ADDR, CMD_WRITEDAC, [(0 >> 4) & 0xFF, (0 << 4) & 0xFF])

            # Set the motor controller status to connected
            RoverStatus.motorControllerStatus = RoverStatus.ready
            
        except(IOError):
            
            # Change the motor controller status to not connected because I2C
            # failed to communicate with the DAC module
            RoverStatus.motorControllerStatus = RoverStatus.notConnected
    
    #------------------------------------------------------------------------------
    # Name:        setMotorSpeed
    # Description: Sets a motor to a given fraction of its top speed from 0
    #              to 1, where 1 is full speed.
    # Arguments:   - speed, fraction of maximum motor speed from 0 to 1
    # Returns:     N/A
    #------------------------------------------------------------------------------
    def setMotorSpeed(self, speed):
    
        # Only attempt to move the motor if the motor controller is
        # actually connected
        if(RoverStatus.motorControllerStatus == RoverStatus.ready):
    
            # Ensure that the speed fraction is between -1 and 1
            if(speed < -1):
                speed = -1
              
            elif(speed > 1):
                speed = 1
        
            # Set the class speed variable
            self.motorSpeed = speed
        
            # Determine if we want forward or reverse
            if(speed > 0):
                self._setDirection(1)
                
            elif(speed < 0):
                self._setDirection(-1)
        
            # Convert the fraction to the integer value between 0 and 4095 that the 
            # DAC expects for its value register
            output = int(abs(speed) * 4095)
            
            try:
            
                # Send the DAC output value to the DAC
                self.I2Cbus.write_i2c_block_data(DAC_ADDR, CMD_WRITEDAC, [(output >> 4) & 0xFF, (output << 4) & 0xFF])
                
            except(IOError):
                print 'ERROR: I2C communications failed'    
                
    #------------------------------------------------------------------------------
    # Name:        _setDirection
    # Description: Sets the rover movement direction to forward (input of 1) 
    #              or reverse (input of -1).
    # Arguments:   - direction, 1 for forward, -1 for reverse
    # Returns:     N/A
    #------------------------------------------------------------------------------           
    def _setDirection(self, direction):
        
        # Only attempt to change direction if the motor controller is
        # actually connected
        if(RoverStatus.motorControllerStatus == RoverStatus.ready):
        
            # If forward
            if(direction == 1):
            
                # Set the right side wheels clockwise
                GPIO.output(RoverStatus.PIN_WHEEL_DIR_RIGHT, 1)
                
                # Set left side wheels counter-clockwise
                GPIO.output(RoverStatus.PIN_WHEEL_DIR_LEFT, 0)
                
            # If reverse
            elif(direction == -1):
            
                # Set the right side wheels counter-clockwise
                GPIO.output(RoverStatus.PIN_WHEEL_DIR_RIGHT, 0)
                
                # Set left side wheels clockwise
                GPIO.output(RoverStatus.PIN_WHEEL_DIR_LEFT, 1)
            
    #------------------------------------------------------------------------------
    # Name:        getRotationRate
    # Description: Measures the rotation rate of the motors using the motor
    #              controller's pulse output. This is a blocking function for now.
    # Arguments:   N/A
    # Returns:     - motor rotation rate in RPM
    #------------------------------------------------------------------------------           
    def getRotationRate(self):

        # Rotation rate variable that will be returned
        rotationRate = 0
    
        # Only attempt to measure speed if the motor controller is
        # actually connected
        if(RoverStatus.motorControllerStatus == RoverStatus.ready):
            
            # Wait for the falling edge of the first pulse
            GPIO.wait_for_edge(RoverStatus.PIN_WHEEL_SPEEDPULSE, GPIO.FALLING)
            
            # Start the timer
            startTime = time.time()
            
            # Wait for the falling edge of the second pulse
            GPIO.wait_for_edge(RoverStatus.PIN_WHEEL_SPEEDPULSE, GPIO.FALLING)
            
            # Stop the timer
            stopTime = time.time()
            
            # Calculate the pulse period
            period = stopTime - startTime
            
            # Calculate the pulse frequency in Hz
            frequency = 1 / period
            
            # Calculate the rate of rotation in RPM as described in the motor 
            # controller datasheet for an 8-pole motor
            rotationRate = 15 * frequency
        
        print 'Motor speed: ', rotationRate
        
        return rotationRate
                
    #------------------------------------------------------------------------------
    # Name:        brakeOn
    # Description: Turns the adaptive braking system on. See _brakeProcess
    #              description for details.
    # Arguments:   N/A
    # Returns:     N/A
    #------------------------------------------------------------------------------           
    def brakeOn(self):

        # Set up a process for braking receiving and start it
        self.brakeProcessInstance = multiprocessing.Process(name='Brake_Process', target=self._brakeProcess)
        self.brakeProcessInstance.start()    
        
    #------------------------------------------------------------------------------
    # Name:        brakeOff
    # Description: Turns the adaptive braking system off.
    # Arguments:   N/A
    # Returns:     N/A
    #------------------------------------------------------------------------------           
    def brakeOff(self):
    
        #if(self.brakeProcess != None):
        
        # Terminate the braking process
        self.brakeProcessInstance.terminate()  
                
    #------------------------------------------------------------------------------
    # Name:        _brakeProcess
    # Description: Attempts to maintain a wheel speed of zero using the 
    #              getRotationRate function and applying motor speed counter to any
    #              direction of unwanted movement. This function starts this 
    #              process in a separate thread.
    # Arguments:   N/A
    # Returns:     N/A
    #------------------------------------------------------------------------------           
    def _brakeProcess(self):

        # Variable for the direction to increment the speed. Multiplies the
        # increment by 1 or -1
        incrementDirection = 1
        
        # Variables for the current and last measurements of the rotation rate
        rotationRateLast = 0
        rotationRateCurrent = 0
    
        while True:
        
            # Move current speed to speed for last iteration
            rotationRateLast = rotationRateCurrent
        
            # Measure the new current motor speed.
            rotationRateCurrent = self.getRotationRate() 
            
            # If last iteration made the speed worse, change increment direction
            if(rotationRateCurrent > rotationRateLast):
                incrementDirection = incrementDirection * -1
                        
            # Increment the motor speed to hopefully counter the movement
            self.motorSpeed = self.motorSpeed + (incrementDirection * MOTOR_SPEED_INCREMENT)
            self.setMotorSpeed(self.motorSpeed)




        