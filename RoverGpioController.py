#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverGpioController.py
#
# PURPOSE: Handles the setup of Raspberry Pi GPIO pins and provides functions
#          interact with them.
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for RPi GPIO control
import RPi.GPIO as GPIO

# Imported to set the connection status of the GPIO controller
import RoverStatus

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class GpioController:

    #--------------------------------------------------------------------------
    # ServoController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
    
        try: 
            # Set GPIO warnings to false
            GPIO.setwarnings(False)
            
            # Get the GPIO pin numbering mode to board
            GPIO.setmode(GPIO.BOARD)   
            
            # Configure the direction and pullup/pulldown of the GPIO
            # pins that are being used.
            GPIO.setup(RoverStatus.PIN_MODEM_IN, GPIO.OUT) 
            GPIO.setup(RoverStatus.PIN_MODEM_OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
            GPIO.setup(RoverStatus.PIN_WHEEL_DIR_LEFT, GPIO.OUT) 
            GPIO.setup(RoverStatus.PIN_WHEEL_DIR_RIGHT, GPIO.OUT) 
            GPIO.setup(RoverStatus.PIN_WHEEL_COAST, GPIO.OUT) 
            GPIO.setup(RoverStatus.PIN_WHEEL_RUNSTOP, GPIO.OUT) 
            GPIO.setup(RoverStatus.PIN_WHEEL_SPEEDPULSE, GPIO.OUT) 

            # Set the GPIO controller status to ready
            RoverStatus.gpioControllerStatus = RoverStatus.ready
        
        except:
            print 'ERROR: Could not initialize GPIO!'
            
            # Set the GPIO controller status to not connected
            RoverStatus.gpioControllerStatus = RoverStatus.notConnected
    
    #--------------------------------------------------------------------------
    # CommandExecutor Destructor
    #--------------------------------------------------------------------------
    def __del__(self):
    
        # Free up the GPIO channel usage
        GPIO.cleanup() 
    
    #--------------------------------------------------------------------------
    # Name:        gpioTogglePin
    # Description: Toggles a specified GPIO pin
    # Arguments:   - pinNum, GPIO pin number to toggle
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def gpioTogglePin(self, pinNum):
    
        # If the GPIO controller is ready
        if(RoverStatus.gpioControllerStatus == RoverStatus.ready):
        
            # Toggle the specified pin
            GPIO.output(pinNum, not GPIO.input(pinNum))
        
    #--------------------------------------------------------------------------
    # Name:        gpioSetPin
    # Description: Sets a specified GPIO pin high or low
    # Arguments:   - pinNum, GPIO pin number to set
    #              - pinState, 1 for high, 0 for low
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def gpioSetPin(self, pinNum, pinState):
    
        # If the GPIO controller is ready
        if(RoverStatus.gpioControllerStatus == RoverStatus.ready):
        
            # Set the specified pin
            GPIO.output(pinNum, pinState)  
            