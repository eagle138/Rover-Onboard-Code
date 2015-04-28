#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverStreamController.py
#
# PURPOSE: Handles the setup of the camera and gstreamer streams and provides
#          functions to configure them
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for command line process execution
import os
import subprocess
import signal

# Imported to set the connection status of the stream controller
import RoverStatus

# Imported to check for USB webcam connections
import os.path

#******************************************************************************
#                              GLOBAL VARIABLES
#******************************************************************************

# Default stream parameters to use on startup
DEFAULT_WIDTH =   640
DEFAULT_HEIGHT =  480
DEFAULT_FPS =     10
DEFAULT_BITRATE = 500000

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class StreamController:

    #--------------------------------------------------------------------------
    # ServoController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
    
        # Set up the variable to hold the camera stream process so that it can 
        # be started or terminated by different functions
        self.cameraProcess = None
        
        # Check if webcam 0 is connected by checking if /dev/video0 exists
        if(os.path.exists('/dev/video0')):
            RoverStatus.webcamStatus[0] = RoverStatus.ready
        else:
            RoverStatus.webcamStatus[0] = RoverStatus.notConnected
        
        # Check if webcam 1 is connected by checking if the /dev/video1 exists
        if(os.path.exists('/dev/video1')):
            RoverStatus.webcamStatus[1] = RoverStatus.ready
        else:
            RoverStatus.webcamStatus[1] = RoverStatus.notConnected
        
        # Set the stream controller status to ready
        RoverStatus.streamControllerStatus = RoverStatus.ready
        
        # Attempt to start the video stream with default parameters
        self.cameraStart(0, 0, DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FPS, DEFAULT_BITRATE)
    
    #--------------------------------------------------------------------------
    # CommandExecutor Destructor
    #--------------------------------------------------------------------------
    def __del__(self):
    
        # Stop the stream processes
        self.cameraStop()

    #--------------------------------------------------------------------------
    # Name:        cameraStart
    # Description: Starts the video camera stream with the input stream 
    #              parameters
    # Arguments:   - cameraNum, camera number to start, starting at 0
    #              - width, width of video stream in pixels
    #              - height, height of video stream in pixels
    #              - fps, framerate of video stream in frames per second
    #              - avgBitrate, average bitrate of video stream in
    #                bits per second
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def cameraStart(self, cameraNum, protocol, width, height, fps, avgBitrate):
    
        # Only attempt to start the camera if it's connected
        if(RoverStatus.webcamStatus[cameraNum] == RoverStatus.ready):
    
            # Assemble the command line string that will be used to start
            # the gstreamer stream
            if(protocol == 0):
                streamCommand = 'gst-launch-1.0 -v -e uvch264src initial-bitrate=%d average-bitrate=%d peak-bitrate=%d iframe-period=3000 \
                                    device=/dev/video%d name=src auto-start=true \
                                    src.vidsrc ! queue ! video/x-h264,width=%d,height=%d,framerate=%d/1 ! h264parse !  rtph264pay config-interval=1 pt=96 ! \
                                    udpsink host=%s port=%d' % (avgBitrate, avgBitrate, avgBitrate, cameraNum, width, height, fps, RoverStatus.controlAddress, RoverStatus.controlVideoPort)
            else:
                streamCommand = 'gst-launch-1.0 -v -e uvch264src initial-bitrate=%d average-bitrate=%d peak-bitrate=%d iframe-period=3000 \
                                    device=/dev/video%d name=src auto-start=true \
                                    src.vidsrc ! queue ! video/x-h264,width=%d,height=%d,framerate=%d/1 ! h264parse !  rtph264pay config-interval=1 pt=96 ! gdppay ! \
                                    tcpserversink host=%s port=%d' % (avgBitrate, avgBitrate, avgBitrate, cameraNum, width, height, fps, RoverStatus.roverAddress, RoverStatus.roverSendPort)
            
            # Stop any currently running stream processes
            self.cameraStop()
            
            try:
            
                # Open /dev/null to redirect gstreamer output that we don't need to see
                FNULL = open(os.devnull, 'w')

                # Execute the command line command, redirect stdout to /dev/null
                self.cameraProcess = subprocess.Popen(streamCommand, shell=True, stdout=FNULL, preexec_fn=os.setsid)
                
                # Set the stream controller status to ready
                RoverStatus.streamControllerStatus = RoverStatus.ready
         
            except:
            
                # Set the stream controller status to not connected
                RoverStatus.streamControllerStatus = RoverStatus.notConnected
                print 'ERROR: Unable to start gstreamer stream!'
            
    #--------------------------------------------------------------------------
    # Name:        cameraStop
    # Description: Stops the video camera stream
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def cameraStop(self):

        # If the gstreamer stream is currently running
        if(self.cameraProcess != None):
        
            # Kill the camera and gstreamer process with the terminate signal
            os.killpg(self.cameraProcess.pid, signal.SIGTERM)

            