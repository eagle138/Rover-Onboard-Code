#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverCommandExecutor.py
#
# PURPOSE: Receives JSON formatted commands, determines what kind of command
#          it is (video, motor/servo control, etc.) and executes it using the
#          rover's interfaces and peripherals. 
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for JSON command parsing
import json

# Imported for rover status variables
import RoverStatus

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class CommandExecutor:

    #--------------------------------------------------------------------------
    # CommandExecutor Constructor
    #--------------------------------------------------------------------------
    def __init__(self, gpioController, servoController, motorController, streamController):
    
        # Save the controller instances to local variables
        self.gpioController = gpioController
        self.servoController = servoController
        self.motorController = motorController
        self.streamController = streamController
        
    #--------------------------------------------------------------------------
    # Name:        execute
    # Description: Parses and executes a JSON formatted command string using 
    #              the rover's interfaces and peripherals
    # Arguments:   - commandString, JSON formatted command string to execute
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def execute(self, commandString):

        #try: 
        
        # Decode the command to get the command type 
        commandData = json.loads(commandString)
        
        # Get the command type from the command key
        commandType = commandData['command']
        print 'Command Received:', commandType
        
        # Command to toggle a GPIO pin
        if(commandType == 'gpiotog'):
            
            # Extract the pin number from the command string
            pinNum = commandData['pin']
            
            # Toggle the specified pin
            self.gpioController.gpioTogglePin(pinNum)
            
        # Command to set a GPIO pin
        elif(commandType == 'gpioset'):
            
            # Extract the pin number and state from the command string
            pinNum = commandData['pin']
            pinState = commandData['state']
            
            # Set the specified pin
            self.gpioController.gpioSetPin(pinNum, pinState)
                
        # Command to start video feed
        elif(commandType == 'camera'):
            
            # Extract the stream parameters from the command string
            cameraNum = commandData['num']
            protocol = commandData['proto']
            width = commandData['w']
            height = commandData['h']
            fps = commandData['fps']
            maxBitrate = commandData['bitrate']
                 
            # Start the video stream   
            self.streamController.cameraStart(cameraNum, protocol, width, height, fps, maxBitrate)
               
        # Command to stop video feed   
        elif(commandType == 'camerastop'):
            
            # Stop the video stream
            self.streamController.cameraStop()

        # Command to set motor speed using DAC  
        elif(commandType == 'motorspeed'):
            
            # Extract the motor speed value from the command string
            motorSpeed = commandData['speed']
            
            # Set the motor speed
            self.motorController.setMotorSpeed(motorSpeed)   
            
        # Command to turn a specific servo
        elif(commandType == 'servo'):
        
            # Move the servo
            self.servoController.setServoAngle(commandData['servonum'], commandData['angle'])  
        
        # Command to enable adaptive brakes
        elif(commandType == 'brakeon'):
        
            # Turn the brakes on
            self.motorController.brakeOn()
            
        # Command to disable adaptive brakes
        elif(commandType == 'brakeoff'):
        
            # Turn the brakes on
            self.motorController.brakeOff()
            
        # Command to steer the rover by turning steering servos
        elif(commandType == 'steer'):
                
            # Extract the steering angle from the command string
            steeringAngle = commandData['angle']
            
            print 'Steering angle:', steeringAngle
                
            # Set the steering servos to the steering angle
            self.servoController.setServoAngle(RoverStatus.SERVO_WHEEL_FRONT_RIGHT, steeringAngle)
            self.servoController.setServoAngle(RoverStatus.SERVO_WHEEL_FRONT_LEFT, steeringAngle)
            
            # Multiply the back servos by -1 since they are mounted the opposite direction
            self.servoController.setServoAngle(RoverStatus.SERVO_WHEEL_BACK_RIGHT, steeringAngle * -1.0)
            self.servoController.setServoAngle(RoverStatus.SERVO_WHEEL_BACK_LEFT, steeringAngle * -1.0)
            
        # Command to move the rover arm by turning arm servos
        elif(commandType == 'clawinc'):
        
            # Increment the claw position
            self.servoController.incrementClawPosition(commandData['dx'], commandData['dy'], commandData['dz'])
            
        # Command to increment a servo's angle
        elif(commandType == 'servoinc'):
        
            # Increment the servo position
            self.servoController.incrementServoAngle(commandData['servonum'], commandData['inc'])    
            
        # Command to set a servo PWM width
        elif(commandType == 'pwm'):
        
            # Set the servo PWM
            self.servoController.setServoPulseWidth(commandData['servonum'], commandData['pw'])               
                
        #except: 
            #print 'ERROR: Unable to execute command!'
             