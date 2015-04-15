#******************************************************************************
#
# VT RoboOps 2015
# Team Vertex
#
# NAME: RoverServoController.py
#
# PURPOSE: Interfaces with the PCA9685 16-channel 12-bit servo controller by 
#          handling its configuration and operation
#
# AUTHOR/DATE: $Author: S. Krauss $ $Date: 2014/11/23 $
# SOURCE/DATE: New
#
# REVISION HISTORY:
# $Log$
# 2014/11/23 STK: Initial version
# 2014/01/12 STK: Fixed variable names and formatting
# 2015/02/14 STK: Added individual servo min and max angles
# 2015/03/17 STK: Added saving of current servo angle and getServoAngle
#******************************************************************************

# Imported for I2C communications
import smbus

# Imported to set the connection status of the servo controller
import RoverStatus

# Imported for kinematics calculations
import math

#******************************************************************************
#                              GLOBAL VARIABLES
#******************************************************************************

# I2C address of PCA9685 servo controller
SERVO_CTRL_ADDR = 0x40

# Addresses of PCA9685 servo controller registers
MODE1_ADDR = 0x00
MODE2_ADDR = 0x01
PRE_SCALE_ADDR = 0xFE
LED0_OFF_L_ADDR = 0x06

# PCA9685 Clock Rate
PCA9685_CLOCK_RATE_HZ = 25000000

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class ServoController:

    #--------------------------------------------------------------------------
    # ServoController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
    
        # Set up the I2C bus
        self.I2Cbus = smbus.SMBus(1)
        
        # Calculate the desired PCA9685 pre-scale value as described in its
        # data sheet
        prescale_value = int(round(PCA9685_CLOCK_RATE_HZ/(4096*RoverStatus.PULSE_REFRESH_RATE_HZ)) - 1.0)
        
        try:
        
            # Configure the servo controller MODE1, MODE2, PRE_SCALE 
            # registers. MODE1 must first be put into sleep mode so that 
            # PRE_SCALE can be changed. Sleep can then be exited.
            self.I2Cbus.write_byte_data(SERVO_CTRL_ADDR, MODE1_ADDR, (1 << 4) | (1 << 5) | (1 << 7))
            self.I2Cbus.write_byte_data(SERVO_CTRL_ADDR, MODE2_ADDR, (1 << 2) | (1 << 3))
            self.I2Cbus.write_byte_data(SERVO_CTRL_ADDR, PRE_SCALE_ADDR, prescale_value)
            self.I2Cbus.write_byte_data(SERVO_CTRL_ADDR, MODE1_ADDR, (1 << 5) | (1 << 7))

            # Set the motor controller status to ready
            RoverStatus.servoControllerStatus = RoverStatus.ready
            
        except(IOError):
            
            # Change the servo controller status to not connected because I2C
            # failed to communicate with the servo controller
            RoverStatus.servoControllerStatus = RoverStatus.notConnected  
          
        # Loop through and set each servo to its default angle
        for servoNum in range(0, 16):
            self.setServoAngle(servoNum, RoverStatus.SERVO_DEFAULT_ANGLE[servoNum])
    
    #--------------------------------------------------------------------------
    # Name:        pulseWidthToAngle
    # Description: Converts pulse width in milliseconds to angle in degrees
    # Arguments:   - pulseWidthMs, pulse width in milliseconds
    # Returns:     - Servo angle in degrees
    #--------------------------------------------------------------------------    
    def pulseWidthToAngle(self, pulseWidthMs):
        
        # Servos usually have the following pulse width to angle distribution:
        #   -90 deg = 1.0 ms
        #     0 deg = 1.5 ms
        #    90 deg = 2.0 ms
        # Any that do not follow this convention will be handled using the 
        # servo angle multiplier setting.
        return (180.0 * pulseWidthMs - 270.0) 
        
    #--------------------------------------------------------------------------
    # Name:        angleToPulseWidth
    # Description: Converts angle in degrees to pulse width in milliseconds
    # Arguments:   - angleDeg, angle in degrees
    # Returns:     - Servo pulse width in milliseconds
    #--------------------------------------------------------------------------    
    def angleToPulseWidth(self, angleDeg):
        
        # Servos usually have the following pulse width to angle distribution:
        #   -90 deg = 1.0 ms
        #     0 deg = 1.5 ms
        #    90 deg = 2.0 ms
        # Any that do not follow this convention will be handled using the 
        # servo angle multiplier setting.
        return ((angleDeg + 270.0)/180.0) 
                
    #--------------------------------------------------------------------------
    # Name:        setServoPulseWidth
    # Description: Outputs the specified pulse width to a servo. Does not
    #              update the current servo angle variable. This is for
    #              debugging and testing only.
    # Arguments:   - servoNum, number of the servo (0-15) to pulse 
    #              - pulseWidthMs, pulse width in milliseconds
    # Returns:     N/A
    #--------------------------------------------------------------------------    
    def setServoPulseWidth(self, servoNum, pulseWidthMs):
        
        # Only attempt to move the servo if the servo controller is
        # actually connected
        if(RoverStatus.servoControllerStatus == RoverStatus.ready):   
                
            # Translate the pulse width into a number of ticks as described by 
            # the PCA9685 manual.
            onTicks = int(round((pulseWidthMs/1000.0) * RoverStatus.PULSE_REFRESH_RATE_HZ * 4096))

            try:
            
                # Set the pulse width by writing consecutively to its LEDn_OFF 
                # and LEDn_ON registers. This can be done because the servo 
                # controller has been set to automatically increment register 
                # address upon write.
                self.I2Cbus.write_i2c_block_data(SERVO_CTRL_ADDR, 
                                                 (LED0_OFF_L_ADDR + servoNum*4), 
                                                 [0x00, 0x00, onTicks, (onTicks >> 8)])
                
            except(IOError):
                print 'ERROR: I2C communications failed' 
        
    #--------------------------------------------------------------------------
    # Name:        setServoAngle
    # Description: Moves a servo to the specified angle in degrees.
    # Arguments:   - servoNum, number of the servo (0-15) to be moved 
    #              - servoAngleDeg, angle to move servo to in degrees
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def setServoAngle(self, servoNum, servoAngleDeg):
    
        # Only attempt to move the servo if the servo controller is
        # actually connected
        if(RoverStatus.servoControllerStatus == RoverStatus.ready):
        
            # Calibrate the servo angle by adding the offset and 
            # multiplying by the angle multiplier. This is no longer the
            # physical angle and servo will end up at but it is the "angle"
            # it must be set to in order to physically reach the input angle
            # taking into account the calibrations.
            servoAngleCalibrated = servoAngleDeg + RoverStatus.SERVO_ZERO_OFFSET[servoNum]
            servoAngleCalibrated = servoAngleCalibrated * RoverStatus.SERVO_ANGLE_MULTIPLIER[servoNum]
            
            # Ensure that the angle is not past the servo limits.
            if(servoAngleCalibrated > RoverStatus.SERVO_MAX_ANGLE[servoNum]):
                servoAngleCalibrated = RoverStatus.SERVO_MAX_ANGLE[servoNum]
                
            elif(servoAngleCalibrated < RoverStatus.SERVO_MIN_ANGLE[servoNum]):
                servoAngleCalibrated = RoverStatus.SERVO_MIN_ANGLE[servoNum]
                
            # Convert the servo angle into a pulse width
            pulseWidthMs = self.angleToPulseWidth(servoAngleCalibrated)
            
            # Translate the pulse width into a number of ticks as described by 
            # the PCA9685 manual.
            onTicks = int(round((pulseWidthMs/1000.0) * RoverStatus.PULSE_REFRESH_RATE_HZ * 4096))

            try:
            
                # Move the servo by writing consecutively to its LEDn_OFF and 
                # LEDn_ON registers. This can be done because the servo 
                # controller has been set to automatically increment register 
                # address upon write.
                self.I2Cbus.write_i2c_block_data(SERVO_CTRL_ADDR, 
                                                 (LED0_OFF_L_ADDR + servoNum*4), 
                                                 [0x00, 0x00, onTicks, (onTicks >> 8)])
                    
                # Save the set angle to the current servo angle variable
                RoverStatus.SERVO_CURRENT_ANGLE[servoNum] = servoAngleDeg
                    
            except(IOError):
                print 'ERROR: I2C communications failed'                   
      
    #--------------------------------------------------------------------------
    # Name:        incrementServoAngle
    # Description: Adds (or subtracts) the specified angle increment to a
    #              given servo.
    # Arguments:   - servoNum, number of the servo (0-15) to be incremented 
    #              - servoAngleInc, increment to move servo to in degrees
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def incrementServoAngle(self, servoNum, servoAngleInc):
    
        # Calculate the new servo angle by adding the increment to the 
        # current servo angle.
        newServoAngle = RoverStatus.SERVO_CURRENT_ANGLE[servoNum] + servoAngleInc
        
        # Set the new servo angle
        self.setServoAngle(servoNum, newServoAngle)
      
    #--------------------------------------------------------------------------
    # Name:        getClawAnglesFromPosition
    # Description: Calculates the angles required for the base elevation, base
    #              azimuth, and elbow angle in order to position the claw at
    #              the input coordinates, relative to the base.
    # Arguments:   - x, x position of the claw relative to base
    #              - y, y position of the claw relative to base
    #              - z, z position of the claw relative to base
    # Returns:     - Angles of arm azimuth, elevation, and elbow in degrees
    #--------------------------------------------------------------------------
    def getClawAnglesFromPosition(self, x, y, z):
        
        # Lengths of arms
        L1 = RoverStatus.ARM_UPPERARM_LENGTH;
        L2 = RoverStatus.ARM_FOREARM_LENGTH;

        # Total length of imaginary line from base to claw
        B = math.sqrt(x*x + y*y + z*z);
        
        # Total length of imaginary line from base to claw in x-y plane
        Bxy = math.sqrt(x*x + y*y);

        # Calculate intermediate angles used to calculate elevation
        q1 = math.atan2(z, Bxy);
        
        # Calculate the interior angle between the B line and the upper arm
        cosq2 = (B*B+L1*L1-L2*L2)/(2*B*L1)
                
        if (cosq2 > 1.0):
            cosq2 = 1.0
        elif (cosq2 < -1.0):
            cosq2 = -1.0
            
        q2 = math.acos(cosq2);

        # Calculate the servo angles from the intermediate values
        angle_azimuth = (math.pi/2) - math.atan2(x, y);
        angle_elevation = q1+q2;
       
        cos_angle_elbow = (L2*L2+L1*L1-B*B)/(2*L1*L2)
        
        if (cos_angle_elbow > 1.0):
            cos_angle_elbow = 1.0
        elif (cos_angle_elbow < -1.0):
            cos_angle_elbow = -1.0
            
        angle_elbow = math.pi - math.acos(cos_angle_elbow);
        
        # Convert angles to degrees
        angle_azimuth = (angle_azimuth * 180 / math.pi)
        angle_elevation = (angle_elevation * 180 / math.pi)
        angle_elbow = (angle_elbow * 180 / math.pi)
        
        return [angle_azimuth, angle_elevation, angle_elbow]
        
    #--------------------------------------------------------------------------
    # Name:        getClawPositionFromAngles
    # Description: Calculates the claw position relative to the base given
    #              the angles of the base azimuth and elevation, and the 
    #              elbow angle.
    # Arguments:   - angle_azimuth, azimuth servo angle in degrees
    #              - angle_elevation, elevation servo angle in degrees
    #              - angle_elbow, elbow servo angle in degrees
    # Returns:     - Position of the claw relative to the base in the same units
    #              as the arm lengths
    #--------------------------------------------------------------------------
    def getClawPositionFromAngles(self, angle_azimuth, angle_elevation, angle_elbow):
    
        # Lengths of arms
        L1 = RoverStatus.ARM_UPPERARM_LENGTH;
        L2 = RoverStatus.ARM_FOREARM_LENGTH;
    
        # Convert angles to radians
        angle_azimuth = angle_azimuth * math.pi / 180
        angle_elevation = angle_elevation * math.pi / 180
        angle_elbow = angle_elbow * math.pi / 180
    
        # Calculate the claw position using forward kinematics
        x = math.cos(angle_azimuth)*(L1*math.cos(angle_elevation)+L2*math.cos(angle_elevation-angle_elbow))
        y = math.sin(angle_azimuth)*(L1*math.cos(angle_elevation)+L2*math.cos(angle_elevation-angle_elbow))
        z = L1*math.sin(angle_elevation)+L2*math.sin(angle_elevation-angle_elbow)
        
        return [x, y, z]

    #--------------------------------------------------------------------------
    # Name:        getCurrentClawPosition
    # Description: Returns the current position of the claw relative to the 
    #              base
    # Arguments:   N/A
    # Returns:     - Current position of the claw relative to the base in the 
    #                same units as the arm lengths
    #--------------------------------------------------------------------------    
    def getCurrentClawPosition(self):
        
        # Get the current servo angles
        azumithAngle = RoverStatus.SERVO_CURRENT_ANGLE[RoverStatus.SERVO_ARM_BASE_AZIMUTH]
        elevationAngle = RoverStatus.SERVO_CURRENT_ANGLE[RoverStatus.SERVO_ARM_BASE_ELEVATION]
        elbowAngle = RoverStatus.SERVO_CURRENT_ANGLE[RoverStatus.SERVO_ARM_ELBOW]
        
        return self.getClawPositionFromAngles(azumithAngle, elevationAngle, elbowAngle)
        
    #--------------------------------------------------------------------------
    # Name:        incrementClawPosition
    # Description: 
    # Arguments:   - dx, x position of the claw relative to base
    #              - dy, y position of the claw relative to base
    #              - dz, z position of the claw relative to base
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def incrementClawPosition(self, dx, dy, dz):
    
        # Get the current coordinates of the claw
        currentClawPosition = self.getCurrentClawPosition()
        
        # Add the claw position increments to the current claw position
        newXPosition = currentClawPosition[0] + dx
        newYPosition = currentClawPosition[1] + dy
        newZPosition = currentClawPosition[2] + dz

        # Calculate the new claw angles from the new claw position
        newClawAngles = self.getClawAnglesFromPosition(newXPosition, newYPosition, newZPosition)
        
        # Set the arm servos to the newly calculated angles
        self.setServoAngle(RoverStatus.SERVO_ARM_BASE_AZIMUTH, newClawAngles[0])
        self.setServoAngle(RoverStatus.SERVO_ARM_BASE_ELEVATION, newClawAngles[1])
        self.setServoAngle(RoverStatus.SERVO_ARM_ELBOW, newClawAngles[2])
        
       
    
        
            