#******************************************************************************
#
# VT RoboOps 2015
# Team Vertex
#
# NAME: RoverGpsController.py
#
# PURPOSE: Interfaces with the Adafruit Ultimate GPS Breakout board and
#          and provides methods to access GPS data. This class is a subclass
#          of the Thread class so that it can gather the stream of data from 
#          the GPS independently. Must call start() function to start
#          the thread and run the GpsController.
#
# AUTHOR/DATE: $Author: S. Krauss $ $Date: 2014/01/14 $
# SOURCE/DATE: New
#
# REVISION HISTORY:
# $Log$
# 2014/01/14 STK: Initial version
#******************************************************************************

# Imported for GPS communications over USB to serial converter
from gps import *

# Imported for subclassing of the Thread class
from threading import Thread

# Imported to set the connection status of the GPS controller
import RoverStatus

# Imported to check for USB GPS connection
import os.path

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class GpsController(Thread):
    
    #--------------------------------------------------------------------------
    # GpsController Constructor
    #--------------------------------------------------------------------------
    def __init__(self):
        
        # Initialize the Thread class
        Thread.__init__(self)
        
        # Check if the GPS module is connected by checking if /dev/ttyUSB0 exists
        if(os.path.exists('/dev/ttyUSB0')):
        
            try:
        
                # Start gathering the GPS data stream
                self.gpsSession = gps(mode=WATCH_ENABLE)
            
                # Create a variable to store the thread's running status
                self.running = False
                
                # Set the GPS controller status to connected
                RoverStatus.gpsControllerStatus = RoverStatus.ready
            
            except:
                
                # Change the GPS controller status to not connected because USB
                # failed to communicate with the GPS module
                RoverStatus.gpsControllerStatus = RoverStatus.notConnected
            
        else:
        
            # Set the GPS controller status to not connected
            RoverStatus.gpsControllerStatus = RoverStatus.notConnected
        
    #--------------------------------------------------------------------------
    # Name:        run
    # Description: Function called when the start() method is called. Loops to
    #              get the GPS data from the buffer.
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def run(self):
        
        # Only attempt to run the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
        
            # Continually loop to gather the GPS data from the buffer
            self.running = True
            while self.running:
                self.gpsSession.next()

    #--------------------------------------------------------------------------
    # Name:        stop
    # Description: Stops the run() method and therefore the GpsController and
    #              its thread.
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def stop(self):
        
        # Set the running variable to false to stop the run method
        self.running = False
    
    #--------------------------------------------------------------------------
    # Name:        getLatitude
    # Description: Returns the most recent Latitude reading from the GPS module
    # Arguments:   N/A
    # Returns:     - most recent GPS Latitude reading in degrees
    #--------------------------------------------------------------------------
    def getLatitude(self):
        
        # Only attempt to read the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
            
            latitude = self.gpsSession.fix.latitude
            
        else:
            
            latitude = 0.0
            
        # Return the Latitude reading
        return latitude
  
    #--------------------------------------------------------------------------
    # Name:        getLongitude
    # Description: Returns the most recent Longitude reading from the GPS 
    #              module
    # Arguments:   N/A
    # Returns:     - most recent GPS Longitude reading in degrees
    #--------------------------------------------------------------------------
    def getLongitude(self):
        
        # Only attempt to read the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
            
            longitude = self.gpsSession.fix.longitude
            
        else:
            
            longitude = 0.0
            
        # Return the Longitude reading
        return longitude
  
    #--------------------------------------------------------------------------
    # Name:        getTime
    # Description: Returns the most recent Time reading from the GPS module
    # Arguments:   N/A
    # Returns:     - most recent GPS Time reading
    #--------------------------------------------------------------------------
    def getTime(self):
        
        # Only attempt to read the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
            
            time = self.gpsSession.fix.time
            
        else:
            
            time = 0.0
            
        # Return the Time reading
        return time
  
    #--------------------------------------------------------------------------
    # Name:        getAltitude
    # Description: Returns the most recent Altitude reading from the GPS module
    # Arguments:   N/A
    # Returns:     - most recent GPS Altitude reading in meters
    #--------------------------------------------------------------------------
    def getAltitude(self):
        
        # Only attempt to read the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
            
            altitude = self.gpsSession.fix.altitude
            
        else:
            
            altitude = 0.0
            
        # Return the Altitude reading
        return altitude

    #--------------------------------------------------------------------------
    # Name:        getSpeed
    # Description: Returns the most recent Speed reading from the GPS module
    # Arguments:   N/A
    # Returns:     - most recent GPS Speed reading in meters per second
    #--------------------------------------------------------------------------
    def getSpeed(self):
        
        # Only attempt to read the GPS controller if it's actually connected
        if(RoverStatus.gpsControllerStatus == RoverStatus.ready):
            
            speed = self.gpsSession.fix.speed
            
        else:
            
            speed = 0.0
            
        # Return the Speed reading
        return speed
