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
#                              CLASS DEFINITION
#******************************************************************************
class StreamController:

    #--------------------------------------------------------------------------
    # ServoController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
    
        # Set up the variable to hold the camera stream processes so that it 
        # can be started or terminated by different functions
        self.cameraProcess = None
        self.audioProcess = None
        
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
        self.cameraStart(0, 0, RoverStatus.DEFAULT_VIDEO_WIDTH, 
                               RoverStatus.DEFAULT_VIDEO_HEIGHT, 
                               RoverStatus.DEFAULT_VIDEO_FPS, 
                               RoverStatus.DEFAULT_VIDEO_BITRATE,
                               RoverStatus.DEFAULT_VIDEO_IFRAME)
        
        # Attempt to start the video stream with default parameters                    
        # self.audioStart()
    
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
    #              - iframeInt, Iframe interval in seconds
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def cameraStart(self, cameraNum, protocol, width, height, fps, avgBitrate, iframeInt):
    
        # Only attempt to start the camera if it's connected
        if(RoverStatus.webcamStatus[cameraNum] == RoverStatus.ready):
    
            # Assemble the command line string that will be used to start
            # the gstreamer stream
            if(protocol == 0):
                streamCommand = 'gst-launch-1.0 -v -e uvch264src initial-bitrate=%d average-bitrate=%d peak-bitrate=%d iframe-period=%d \
                                    device=/dev/video%d name=src auto-start=true \
                                    src.vidsrc ! queue ! video/x-h264,width=%d,height=%d,framerate=%d/1 ! h264parse !  rtph264pay config-interval=1 pt=96 ! \
                                    udpsink host=%s port=%d' % (avgBitrate, avgBitrate, avgBitrate, iframeInt, cameraNum, width, height, fps, RoverStatus.controlAddress, RoverStatus.controlVideoPort)

            else:
                streamCommand = 'gst-launch-1.0 -v -e uvch264src initial-bitrate=%d average-bitrate=%d peak-bitrate=%d iframe-period=%d \
                                    device=/dev/video%d name=src auto-start=true \
                                    src.vidsrc ! queue ! video/x-h264,width=%d,height=%d,framerate=%d/1 ! h264parse !  rtph264pay config-interval=1 pt=96 ! gdppay ! \
                                    tcpserversink host=%s port=%d' % (avgBitrate, avgBitrate, avgBitrate, iframeInt, cameraNum, width, height, fps, RoverStatus.roverAddress, RoverStatus.roverSendPort)
            
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
    # Name:        audioStart
    # Description: Starts the audio stream
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def audioStart(self):
    
        # Only attempt to start the camera if it's connected
        if(RoverStatus.webcamStatus[0] == RoverStatus.ready):
    
            # Assemble the command line string that will be used to start
            # the gstreamer stream
            streamCommand = 'gst-launch-1.0 alsasrc device=hw:1 ! \
                             "audio/x-raw,channels=2,rate=48000,depth=16" ! \
                             audioconvert ! audioresample ! alawenc ! rtppcmapay ! \
                             udpsink host=%s port=%d' % (RoverStatus.controlAddress, RoverStatus.controlAudioPort)
                                
            # Stop any currently running stream processes
            self.audioStop()
            
            try:
            
                # Open /dev/null to redirect gstreamer output that we don't need to see
                FNULL = open(os.devnull, 'w')

                # Execute the command line command, redirect stdout to /dev/null
                self.audioProcess = subprocess.Popen(streamCommand, shell=True, stdout=FNULL, preexec_fn=os.setsid)
                
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
            
    #--------------------------------------------------------------------------
    # Name:        audioStop
    # Description: Stops the audio stream
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def audioStop(self):

        # If the gstreamer stream is currently running
        if(self.audioProcess != None):
        
            # Kill the camera and gstreamer process with the terminate signal
            os.killpg(self.audioProcess.pid, signal.SIGKILL)

            