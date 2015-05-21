#******************************************************************************
# VT RoboOps 2015 - Team Vertex
# 
# NAME:    RoverSocket.py
#
# PURPOSE: Module containing the Rover's socket class for establishing a TCP
#          connection and sending/receiving data. Can be used to create a socket
#          that listens for connections, or one that attempts to connect to
#          a peer
#
# AUTHOR:  S. Krauss
#******************************************************************************

# Imported for python's TCP socket class
import socket

#******************************************************************************
#                               FUNCTIONS
#******************************************************************************

#------------------------------------------------------------------------------
# Name:        sendCommandString
# Description: Sends the given command string to the given address over
#              a TCP connection. Handles connection opening and closing.
#              Raises a TimeoutError if the connection times out.
# Arguments:   - address, peer IP address to send command string to
#              - port, peer port address to send command string to
#              - message, message to be sent to peer
#              - timeoutMs, message connection timeout in milliseconds
# Returns:     N/A
#------------------------------------------------------------------------------
def sendCommandString(address, port, message, timeoutMs):

    try:
    
        # Create the socket for command sending
        sendSocket = TcpSocket()

        # Set the connection timeout
        sendSocket.setTimeout(timeoutMs/1000)
            
        # Connect to the target address and port
        sendSocket.connect(address, port)

        # Send the a command string
        sendSocket.send(message, len(message))

        # Close the socket
        sendSocket.disconnect()
        
    except(socket.timeout):
    
        # Close the socket
        sendSocket.disconnect()

        raise socket.timeout
    
    except:
    
        print "ERROR: Connection to %s:%d refused!" % (address, port)
    
        # Close the socket
        sendSocket.disconnect()

#******************************************************************************
#                              CLASS DEFINITION
#******************************************************************************
class TcpSocket:

    #--------------------------------------------------------------------------
    # TcpSocket Constructor
    #--------------------------------------------------------------------------
    def __init__(self, sock=None):
        
        # If no socket is specified, create a new one
        if sock is None:       
            # Create a new socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
        else:     
            # Use the given socket
            self.sock = sock
        
    #--------------------------------------------------------------------------
    # Name:        setTimeout
    # Description: Sets the socket's connection timeout time in seconds
    # Arguments:   - timeoutTime, connection timeout time in seconds
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def setTimeout(self, timeoutTime):
        
        # Set the socket timeout value
        self.sock.settimeout(timeoutTime)
        
    #--------------------------------------------------------------------------
    # Name:        open
    # Description: Binds the socket to the given address and port and listens 
    #              or connections.
    # Arguments:   - host, host IP address of the socket
    #              - port, port to bind the socket to
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def open(self, host, port):
        
        # Set the socket to be reusable
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind the socket to the given host address and port
        self.sock.bind((host, port))
        
        # Begin listening for connections on the socket
        self.sock.listen(5)
        
    #--------------------------------------------------------------------------
    # Name:        accept
    # Description: Accepts an incoming TCP connection. This operation blocks
    #              code operation until a connection is accepted.
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def accept(self):
        
        # Accept a TCP connection. This is a blocking operation.
        client_socket, client_address = self.sock.accept()    
        
    #--------------------------------------------------------------------------
    # Name:        connect
    # Description: Attempts to connect to the given host address and port.
    #              Times out if the socket's timeout time elapses before a 
    #              connection is made.
    # Arguments:   - host, host IP address to connect to
    #              - port, port to connect to
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def connect(self, host, port):
    
        # Attempt to connect to the given host address and port
        self.sock.connect((host, port))
        
    #--------------------------------------------------------------------------
    # Name:        disconnect
    # Description: Closes the TCP socket. The socket is gone after this point 
    #              and a new socket must be established to use it.
    # Arguments:   N/A
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def disconnect(self):
        
        # Close the socket
        self.sock.close()

    #--------------------------------------------------------------------------
    # Name:        send
    # Description: Sends the given message to the TCP peer if a connection has
    #              been made.
    # Arguments:   - message, message to be sent to peer
    #              - length, length of message to be sent
    # Returns:     N/A
    #--------------------------------------------------------------------------
    def send(self, message, length):
        
        # Variable to store the number of bytes successfully sent
        bytesSent = 0
        
        # Loop until all bytes are sent
        while bytesSent < length:
        
            # Send some more bytes
            sent = self.sock.send(message[bytesSent:])
            
            # Return an error if the socket connection was somehow broken
            if sent == 0:
                raise RuntimeError("Socket connection broken!")
                
            # Increment the total number of bytes sent
            bytesSent = bytesSent + sent

    #--------------------------------------------------------------------------
    # Name:        receiveFixedLength
    # Description: Receives a fixed length message from the TCP peer if
    #              if a connection has been made.
    # Arguments:   - length, length of message to be received in bytes
    # Returns:     - message received from peer
    #--------------------------------------------------------------------------
    def receiveFixedLength(self, length):

        # Array of all data chunks received
        receivedMessageChunks = []
        
        # Total number of bytes received
        bytes_received = 0
        
        # Accept a connection from client
        client_socket, client_address = self.sock.accept()
        
        # Loop until all bytes are read
        while bytes_received < length:
        
            # Receive some more bytes from the client
            chunk = client_socket.recv(min(length - bytes_received, 2048))
            
            # Return an error if the socket connection was somehow broken
            if chunk == '':
                raise RuntimeError("Socket connection broken!")
            
            # Add the received bytes to the array
            receivedMessageChunks.append(chunk)
            
            # Increment the total number of bytes received
            bytes_received = bytes_received + len(chunk)
            
        # Return the fully received message by joining array into a string    
        return ''.join(receivedMessageChunks)

    #--------------------------------------------------------------------------
    # Name:        receiveTerminatingChar
    # Description: Receives a message from the TCP peer until the specified 
    #              terminating character has been read if a connection has been 
    #              made.
    # Arguments:   - terminatingChar, message terminating character
    # Returns:     - message received from peer
    #--------------------------------------------------------------------------
    def receiveTerminatingChar(self, terminatingChar):
    
        try:
            # The data chunk just received
            currentMessageChunk = ''
            
            # Array of all data chunks received
            receivedMessageChunks = []
            
            # Accept a connection from client
            clientSocket, clientAddress = self.sock.accept()
            
            # Loop until the terminating character is read
            while terminatingChar not in currentMessageChunk:
            
                # Receive some more bytes from the client
                currentMessageChunk = clientSocket.recv(2048)
                
                # Return an error if the socket connection was somehow broken
                if currentMessageChunk == '':
                    raise RuntimeError("Socket connection broken!")
                
                # Add the received bytes to the array
                receivedMessageChunks.append(currentMessageChunk)
                
            # Return the fully received message by joining array into a string    
            return ''.join(receivedMessageChunks)
        
        except:
            print 'ERROR: TCP receive terminated.'
        