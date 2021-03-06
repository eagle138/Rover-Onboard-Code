#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverStatus.py
#
# PURPOSE: Holds important rover setting and status variables.
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for python's TCP socket class
import socket

#******************************************************************************
#                              GLOBAL ROVER SETTINGS
#******************************************************************************

#------------------------------------------------------------------------------
# Rover General Settings
#------------------------------------------------------------------------------

# Option to print incoming commands on the rover control code console display
PRINT_COMMANDS = True

#------------------------------------------------------------------------------
# Rover Device Status Variables
#------------------------------------------------------------------------------

# Status indication variables
notConnected = 'Not connected'
ready = 'Ready'

# Rover status variables
gpioControllerStatus   = notConnected
servoControllerStatus  = notConnected
motorControllerStatus  = notConnected
streamControllerStatus = notConnected
gpsControllerStatus    = notConnected
webcamStatus = [notConnected, notConnected, notConnected]

#------------------------------------------------------------------------------
# Rover Network Settings
#------------------------------------------------------------------------------

# IP address of the computer running the control software as set up in Hamachi
controlAddress = '25.145.186.98' # Deimos
#controlAddress = '25.5.127.28' # Umbra
#controlAddress = '25.92.145.123' # Control Room

# IP address of the rover as set up by Hamachi
#roverAddress = '25.129.69.92'
roverAddress = '25.125.146.63'


# Ports used by the rover and control server for command sending and receiving
roverListenPort =   5000
roverSendPort =     5001
controlListenPort = 5001
controlSendPort =   5000
controlVideoPort =  1338
controlAudioPort =  1339

# Interval at which to send heartbeats in milliseconds
heartbeatIntervalMs = 2000 

# Number of connection to control timeouts before video and audio streams
# are terminated in case the network is being flooded by them
CONNECTION_TIMEOUTS = 2 

#------------------------------------------------------------------------------
# Rover GPIO Settings
#------------------------------------------------------------------------------

# Raspberry PI 2 GPIO pin layout
PIN_MODEM_IN         = 7
PIN_MODEM_OUT        = 11
PIN_WHEEL_DIR_LEFT   = 12
PIN_WHEEL_DIR_RIGHT  = 13
PIN_WHEEL_COAST      = 15
PIN_WHEEL_RUNSTOP    = 16
PIN_WHEEL_SPEEDPULSE = 18
PIN_GPS_FIX          = 19

#------------------------------------------------------------------------------
# Rover Servo Settings
#------------------------------------------------------------------------------

# Rover wheel servo numbers
SERVO_WHEEL_FRONT_RIGHT   = 0
SERVO_WHEEL_FRONT_LEFT    = 1
SERVO_WHEEL_BACK_RIGHT    = 2
SERVO_WHEEL_BACK_LEFT     = 3

# Rover arm servo numbers
SERVO_ARM_BASE_AZIMUTH    = 4
SERVO_ARM_BASE_ELEVATION  = 5
SERVO_ARM_ELBOW           = 6
SERVO_ARM_WRIST_ELEVATION = 7
SERVO_ARM_WRIST_ROTATION  = 8
SERVO_ARM_CLAW            = 9

# Rover camera servo numbers
SERVO_CAMERA_MAST         = 10
SERVO_CAMERA_AZIMUTH      = 11
SERVO_CAMERA_ELEVATION    = 12

# Servo pulse refresh rate in Hz
PULSE_REFRESH_RATE_HZ = 60

# Servo minimum and maximum angles for each of the 16 servos
SERVO_MIN_ANGLE = [-90, -90, -90, -90, -180, -180, -180, -32, -180, -180, -90, -90, -90, -90, -90, -90]
SERVO_MAX_ANGLE = [ 90,  90,  90,  90,  180,  180,  180,  142,  180,  180,  90,  90,  90,  90,  90,  90]

# Servo default and current angle for each of the 16 servos. The default 
# position is the position they will be moved to upon rover start-up.
SERVO_DEFAULT_ANGLE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
SERVO_CURRENT_ANGLE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Servo angle multiplier to handle servos that don't follow the pulse width
# conventions used by the servo controller code. Also handles axis orientation
# by allowing for negative multiplication.
SERVO_ANGLE_MULTIPLIER = [1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# Angle that each servo is offset from zero when pulse width is set
# to 1.5 ms. This is to correct for "incorrect" mounting of parts to servo.
# When set to this angle, the parts connected to the servo will be
# at zero degrees in their own axes but the servo will not be at its own zero.
SERVO_ZERO_OFFSET = [5.0, 9.0, 4.0, 81.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -69.0, 0.0, 0.0, 0.0]

# Rover arm segment lengths
ARM_UPPERARM_LENGTH = 10.0 # inches
ARM_FOREARM_LENGTH  = 9.5  # inches

#------------------------------------------------------------------------------
# Rover Data Stream Settings
#------------------------------------------------------------------------------

# Default video stream parameters to use on startup
DEFAULT_VIDEO_WIDTH =   640    # pixels
DEFAULT_VIDEO_HEIGHT =  480    # pixels
DEFAULT_VIDEO_FPS =     30     # frames per second
DEFAULT_VIDEO_BITRATE = 500000 # bits per second
DEFAULT_VIDEO_IFRAME =  1      # seconds
