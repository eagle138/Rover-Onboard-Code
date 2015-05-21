#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverStreamController.py
#
# PURPOSE: Handles the set-up of live gstreamer video and audio streams over 
#          the network and manages their settings.
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

import threading

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
        self.videoProcess = None
        self.audioProcess = None
        
        # Check if webcam 0 is connected by checking if /dev/video0 exists
        if(os.path.exists('/dev/video0')):
            RoverStatus.webcamStatus[0] = RoverStatus.ready
        else:
            RoverStatus.webcamStatus[0] = RoverStatus.notConnected
        
        # Check if webcam 1 is connected by checking if /dev/video1 exists
        if(os.path.exists('/dev/video1')):
            RoverStatus.webcamStatus[1] = RoverStatus.ready
        else:
            RoverStatus.webcamStatus[1] = RoverStatus.notConnected
        
        # Check if webcam 2 is connected by checking if /dev/video2 exists        
        if(os.path.exists('/dev/video2')):
            RoverStatus.webcamStatus[2] = RoverStatus.ready
        else:
            RoverStatus.webcamStatus[2] = RoverStatus.notConnected
        
        # Set the stream controller status to ready
        RoverStatus.streamControllerStatus = RoverStatus.ready
        
        # Attempt to start the video stream with default parameters
        self.cameraStart( 0, 
                          RoverStatus.DEFAULT_VIDEO_WIDTH, 
                          RoverStatus.DEFAULT_VIDEO_HEIGHT, 
                          RoverStatus.DEFAULT_VIDEO_FPS, 
                          RoverStatus.DEFAULT_VIDEO_BITRATE,
                          RoverStatus.DEFAULT_VIDEO_IFRAME)
        
        # Attempt to start the audio stream with default parameters                    
        #self.audioStart()
    
    #--------------------------------------------------------------------------
    # CommandExecutor Destructor
    #--------------------------------------------------------------------------
    def __del__(self):
    
        # Stop the stream processes
        self.videoStop()
        self.audioStop()

    #--------------------------------------------------------------------------
    # Name:        cameraStart
    # Description: Starts the video camera stream with the input stream 
    #              parameters
    # Arguments:   - cameraNum, camera number to start, starting at 0
    #              - width, width of video stream in pixels
    #              - height, height of video stream in pixels
    #              - framerate, framerate of video stream in frames per second
    #              - avgBitrate, average bitrate of video stream in
    #                bits per second
    #              - iframeInt, Iframe interval in seconds
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def cameraStart(self, cameraNum, width, height, framerate, avgBitrate, iframeInt):
    
        # Only attempt to start the camera if it's connected
        if(RoverStatus.webcamStatus[cameraNum] == RoverStatus.ready):
    
            # Assemble the command line string that will be used to start
            # the gstreamer video stream over UDP
            streamCommand = 'gst-launch-1.0 -v -e uvch264src initial-bitrate=%d average-bitrate=%d peak-bitrate=%d iframe-period=%d device=/dev/video%d name=src auto-start=true src.vidsrc \
                                ! queue \
                                ! video/x-h264,width=%d,height=%d,framerate=%d/1 \
                                ! udpsink host=%s port=%d' \
                                % (avgBitrate, avgBitrate, avgBitrate, iframeInt, cameraNum, width, height, framerate, RoverStatus.controlAddress, RoverStatus.controlVideoPort)

            # Stop any currently running video stream processes
            self.videoStop()
            
            try:
            
                # Open /dev/null to redirect gstreamer output that we don't need to see
                FNULL = open(os.devnull, 'w')

                # Execute the command line command, redirect stdout to /dev/null
                self.videoProcess = subprocess.Popen(streamCommand.split(), stdout=FNULL, stderr=FNULL)
                
                # Set the stream controller status to ready
                RoverStatus.streamControllerStatus = RoverStatus.ready
         
            except:
            
                #Set the stream controller status to not connected
                RoverStatus.streamControllerStatus = RoverStatus.notConnected
                print 'ERROR: Unable to start gstreamer video stream!'
            
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
            # the gstreamer audio stream over UDP
            streamCommand = 'gst-launch-1.0 alsasrc device=hw:1 \
                                ! audioconvert \
                                ! audioresample \
                                ! alawenc \
                                ! rtppcmapay \
                                ! udpsink host=%s port=%d' \
                                % (RoverStatus.controlAddress, RoverStatus.controlAudioPort)
                                
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
                print 'ERROR: Unable to start gstreamer audio stream!'                 
            
    #--------------------------------------------------------------------------
    # Name:        videoStop
    # Description: Stops the video camera stream
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def videoStop(self):
    
        # If the gstreamer stream is currently running
        if(self.videoProcess != None):
        
            # Kill the camera and gstreamer process with the terminate signal
            #os.killpg(self.videoProcess.pid, signal.SIGKILL)
            self.videoProcess.kill()
            #subprocess.Popen('sudo kill $(pidof gst-launch-1.0)', shell=True)
     
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

            