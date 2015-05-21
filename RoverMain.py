#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverMain.py
#
# PURPOSE: Initiates the command sending and receiving processes. The sending
#          process sets up a socket for the rover to send heartbeats to the
#          control software at a defined interval. These heartbeats contain
#          information about the rover. The receive process sets up a socket
#          that the rover uses to listen for commands from the control software.
#          When a command is received, it is sent to the command executor to 
#          execute the command by interfacing with hardware and peripherals.
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for rover status variables
import RoverStatus

# Imported for set-up of communication sockets
import RoverSocket

# Imported for command execution
from RoverCommandExecutor import CommandExecutor

# Imported for control of rover components
from RoverGpioController import GpioController
from RoverServoController import ServoController
from RoverMotorController import MotorController
from RoverStreamController import StreamController
from RoverGpsController import GpsController

# Imported for creation of command receiving and heartbeat sending threads
import multiprocessing

# Imported for timing of heartbeat interval
import time

# Imported for formation of JSON heartbeat commands
import json

# Imported for killing of gstreamer processes
from subprocess import call

# Imported for clearing of console
import os

# Imported for socket timeout exception
import socket

# Imported for CPU statistics
import psutil

#******************************************************************************
#                             FUNCTION DEFINITIONS
#******************************************************************************

#------------------------------------------------------------------------------
# Name:        receiveCommandsProcess
# Description: Waits for a TCP socket connection, accepts the connection,
#              handles any commands sent through it, then closes the socket
# Arguments:   N/A
# Returns:     N/A
#------------------------------------------------------------------------------
def receiveCommandsProcess():   
    
    print 'Starting command receive process...'
    
    # Import the controllers from main
    global gpioController
    global servoController
    global motorController
    global streamController
    
    # Create an instance of the command executor and give it the instances of
    # the controllers
    commandExecutor = CommandExecutor(gpioController, servoController, motorController, streamController) 
    
    #  Create and open the socket the rover will be listening on
    receiveSocket = RoverSocket.TcpSocket()
    receiveSocket.open(RoverStatus.roverAddress, RoverStatus.roverListenPort)

    # Continuously loops to check for TCP socket connections and accept
    # any commands received
    while True:
    
        # Receive a command from the server. JSON commands are terminated by }
        receivedCommand = receiveSocket.receiveTerminatingChar('}')
        
        # Send the command to the command executor to be executed
        commandExecutor.execute(receivedCommand)
        
#------------------------------------------------------------------------------
# Name:        sendHeartbeatProcess
# Description: Sends a heartbeat to the control server using TCP at at the 
#              interval defined by RoverStatus.heartbeatIntervalMs. Closes the 
#              socket after each heartbeat
# Arguments:   N/A
# Returns:     N/A
#------------------------------------------------------------------------------
def sendHeartbeatProcess():   
    
    print 'Starting heartbeat send process...'
    
    # Import the controllers from main
    global gpsController
    global motorController
    global streamController
    
    # Variable to store the number of times the connection to control has
    # timed out in a row.
    numTimeouts = 0
    
    # Start the GPS controller thread
    gpsController.start()
    
    # Command strings to send from the rover to itself in case of disconnection
    videostopString = "{\"command\":\"videostop\"}"
    audiostopString = "{\"command\":\"audiostop\"}"
    motorstopString = "{\"command\":\"motorspeed\", \"speed\": 0.0}"
    
    # Continuously loops to send heartbeat over TCP at the defined interval
    while True:
    
        try:
      
            # Sleep thread for heartbeat send interval. Done at start do that
            # process will sleep even if the connection times out.
            time.sleep(RoverStatus.heartbeatIntervalMs/1000)
      
            # Gather the data to be sent over the heartbeat
            latitude = gpsController.getLatitude()
            longitude = gpsController.getLongitude()
            altitude = gpsController.getAltitude()
            speed = gpsController.getSpeed()
            cpu_usage = psutil.cpu_percent()
        
            # If the CPU usage is 100%, stop the video and audio streams
            # that are causing it so that the quality can be lowered.
            if (cpu_usage == 100):

                RoverSocket.sendCommandString(RoverStatus.roverAddress, 
                                              RoverStatus.roverListenPort, 
                                              videostopString, 
                                              RoverStatus.heartbeatIntervalMs)
                                              
                RoverSocket.sendCommandString(RoverStatus.roverAddress, 
                                              RoverStatus.roverListenPort, 
                                              audiostopString, 
                                              RoverStatus.heartbeatIntervalMs)
        
            # Form the  JSON formatted heartbeat command string
            heartbeatString = json.dumps({'command': 'heartbeat', 
                                          'cpu': cpu_usage,
                                          'lat': latitude, 
                                          'long': longitude, 
                                          'alt': altitude, 
                                          'speed': speed})
    
            RoverSocket.sendCommandString(RoverStatus.controlAddress, 
                                          RoverStatus.controlListenPort, 
                                          heartbeatString, 
                                          RoverStatus.heartbeatIntervalMs)
                                          
            # Reset the number of timeouts upon a successful connection
            numTimeouts = 0
        
        # Called when ctrl+c is hit to exit program
        except(KeyboardInterrupt, SystemExit):
        
            # Call the rover shutdown routine
            roverShutdown()
        
        except(socket.timeout):
        
            print 'ERROR: Connection to control timed out.'
            
            # Increment the number of timeouts toward the limit set in
            # the settings file. Will be reset upon successful connection.
            numTimeouts += 1 
            
            # Send a command from the rover to itself to turn off the motors
            # in case the connection has been severed while we're still
            # moving. We don't want the rover running away out of control.
            RoverSocket.sendCommandString(RoverStatus.roverAddress, 
                                          RoverStatus.roverListenPort, 
                                          motorstopString, 
                                          RoverStatus.heartbeatIntervalMs)
                                          
            # If the number of timeouts of the connection to the control 
            # software has hit the set limit 
            if (numTimeouts >= RoverStatus.CONNECTION_TIMEOUTS):

                # Send commands to shut off the audio and video streams to 
                # make sure they are not flooding the network causing
                # the loss of control.
                RoverSocket.sendCommandString(RoverStatus.roverAddress, 
                                              RoverStatus.roverListenPort, 
                                              videostopString, 
                                              RoverStatus.heartbeatIntervalMs)
                                              
                RoverSocket.sendCommandString(RoverStatus.roverAddress, 
                                              RoverStatus.roverListenPort, 
                                              audiostopString, 
                                              RoverStatus.heartbeatIntervalMs)
                                              
        except:

            print 'ERROR: Send process general exception caught!'
                   
#------------------------------------------------------------------------------
# Name:        roverShutdown
# Description: Terminates the command processor and its processes
# Arguments:   N/A
# Returns:     N/A
#------------------------------------------------------------------------------
def roverShutdown():   
        
    global streamController    
        
    # Shut down the video and audio streams    
    streamController.videoStop()
    streamController.audioStop()     
        
    # close the receive and send processes
    sendProcess.terminate()
    receiveProcess.terminate()
    
    print "\nRover shut down."    
    print '--------------------------------------------------------------------------------'

#******************************************************************************
#                               MAIN PATH
#******************************************************************************
if __name__ == '__main__':

    # Clear the console
    os.system("clear")

    print 'Rover Main State Machine -------------------------------------------------------'
    print 'Robo-Ops 2015 - Virginia Tech - Team Vertex'
    print 'Initiating rover startup sequence...'
    print 'Local IP address:  ', RoverStatus.roverAddress
    print 'Control IP address:', RoverStatus.controlAddress
    print ' '
    
    # Initialize the rover interface controllers
    gpioController = GpioController()
    motorController = MotorController()
    servoController = ServoController()
    streamController = StreamController()
    gpsController = GpsController()
   
    print 'GPIO   controller...', RoverStatus.gpioControllerStatus
    print 'Servo  controller...', RoverStatus.servoControllerStatus
    print 'Motor  controller...', RoverStatus.motorControllerStatus
    print 'Stream controller...', RoverStatus.streamControllerStatus
    print 'GPS    controller...', RoverStatus.gpsControllerStatus      
    print 'USB Webcam 0...     ', RoverStatus.webcamStatus[0]
    print 'USB Webcam 1...     ', RoverStatus.webcamStatus[1]
    print 'USB Webcam 2...     ', RoverStatus.webcamStatus[2]     
    print ' '
    
    # Initiate separate processes for command receiving and heartbeat sending
    try:
      
        # Set up a process for command receiving and start it
        receiveProcess = multiprocessing.Process(name='Receive_Process', target=receiveCommandsProcess)
        receiveProcess.start()
        
        # Set up a process for heartbeat sending and start it
        sendProcess = multiprocessing.Process(name='Send_Process', target=sendHeartbeatProcess)
        sendProcess.start()
        
        # Join the main thread to the receive thread since the main 
        # thread is finished setting everything up
        receiveProcess.join()
        
    # Called when ctrl+c is hit to exit program
    except(KeyboardInterrupt, SystemExit):
        roverShutdown()
        
    except:
       print "ERROR: Multiprocessing failed."
