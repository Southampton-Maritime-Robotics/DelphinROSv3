#!/usr/bin/python

'''
Possibly, a node to interface with sonar

This node may not functioning!

# TODO
- assign a unique name to the sonar serial port in udev rule.

'''

import rospy
import serial
import time
import numpy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from hardware_interfaces.msg import sonar	

################################################################
def sonarTalker():
    # defines and sends the ping trigger (mtSendData) to the sonar. 
    global serialPort 
    #trigger = [64, 48, 48, 48, 67, 12, 0, 255, 2, 7, 25, 128, 255, 0, 0, 0, 0, 10] # What message should look like with fixed parameters but created below
    
    # construct message using parameters/constants
    AsciiBlock = [64, 48, 48, 48, 67]           # @000C - ASCII header to msg
    hexLength = [12, 0]                         # Length of block in 2 bit int
    SID = rospy.get_param("/SID")               # Source transmission node
    DID = rospy.get_param("/DID")               # Msg destination node
    count = 7                                   # Number of bytes following count byte - excluding LF character 
    msgNum = 25                                 # Msg identification number
    msgSeq = rospy.get_param("/msgSeq")         # Default for this message
    Node = rospy.get_param("/Node")             # Repeat of SID, NOT WHAT IT SAYS IN THE MANUAL
    currentTime = [0,0,0,0]                     # Current time for sonar, not required for imaging as signals not time stamped
    LF = 10                                     # End of line character

    mtSendData = AsciiBlock + hexLength + [SID, DID, count, msgNum, msgSeq, Node] + currentTime + [LF] # Construct mtSendData message
    mtSendData = ''.join([chr(char) for char in mtSendData[:]])  # Converts from string to hex
    serialPort.write(mtSendData)                                 # Send mtSendData message to sonar

    print "Sending ping trigger to sonar - mtSendData:"
    print mtSendData
    print "--------"

################################################################
def sonarListener():
    # reads in the ping reply from the sonar (mtHeadData), determines the message type and publishes the ping data if applicable
    global serialPort
    #print "No. of chars in buffer:",serialPort.inWaiting()
    
    timeOut = 2                                                                 # Timeout for data to appear in buffer (time from ping until data recieved by node)
    
    startTime = time.time()                                                     # Record start time

    while serialPort.inWaiting() < 12:                                          # TEST WHETHER 12 SHOULD BE 13!!! ################# # Waiting until number of bytes is the length of the message header
        time.sleep(0.001)
        if startTime+timeOut < time.time():	                                    # If timeout then return 0
            serialPort.flushInput()
            return 0                                                                
    
    if serialPort.inWaiting() > 12:         
        headerData = serialPort.read(13)                                        # Read 13 bytes of data (length of header)

        headerData = numpy.fromstring(headerData, dtype=numpy.uint8)            # Converts from string to UINT8 numpy array

        print "Data from sonar: mtHeadData:"                                    
        print headerData
        print "--------"

        if headerData[0] == 64:                                                 # Every message should start with an '@' (ASCII code 64)
            msgType = headerData[10]                                            # See manual
            print "Message type:  ",msgType
           
            # From sonarPing.m: get the length of the rest of the data (see catherine harris for sonarping.m!)
            # calulate from mtAlive 22 bytes in total, header[5:7] = 16 bytes and we load in 13 bytes initially
            # thus number to remove = 16 - (22-13) = 7
            # headerData[5]+headerData[6]*256 = simulating Matlab typecast function - converting 2 uint8s to 1 uint16  
            msgLength = (headerData[5]+(headerData[6]*256))-7
            print msgLength
            print serialPort.inWaiting()

            startTime = time.time()                                             # Reset time zero for next timeout

            while serialPort.inWaiting()<msgLength:                             # Makes sure full message is in buffer within timeout
                time.sleep(0.001)
                if startTime+timeOut < time.time():
                    print "Timeout error"
                    serialPort.flushInput()
                    return 0

            msgData = serialPort.read(serialPort.inWaiting())                   # Read rest of message
            msgData = numpy.fromstring(msgData, dtype=numpy.uint8)              # Convert to numpy array of UINT8
            print msgData
           
            if msgData[-1] != 10:                                               # If the message does not end in an '\LF' then return 0
                print "Message error: missing LF character" 
                serialPort.flushInput()
                return 0

            if msgType == 1:                                                    # Each if/elseif statement defines how to process each message type
                print "mtVersionData"
                # mtVersionData                                                 # mtVersionData == 1 -> not currently used
            elif msgType == 2:
                print "mtHeadData"
                
                # Used only for debugging -> visualising sonar data #
                transBearing = msgData[27]+(msgData[28]*256)                    # Converts 2 x UINT8 characters into 1 x UINT16
                transBearing = (transBearing / 6400.0) *360                     # Converts from 1/16 of a gradian to degrees 
                print transBearing
                    
                msgData = numpy.concatenate((headerData,msgData))               # Joins header and message data into one structure
                    
                msgData = ''.join([chr(char) for char in msgData[:]])           # Converts from numpy array into string so it can be published
                pub.publish(str(msgData))

            elif msgType == 4:                                                  # mtAlive == 4 -> not currently processed
                print "mtAlive"
                return 4
            else:
                print "Error: Unknown message type"


        else:
            print "Error: sonar message does not start with the required '@'" 
            return 0
            
    
################################################################    
def setupSonar():
    # Sends the setup command (mtHeadCommand) to the sonar #
    
    global serialPort 
    
    # Serial port settings for Micron Sonar #
    serialPort = serial.Serial(port='/dev/ttyS3', baudrate='115200') 
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print "Setup sonar: mtHeadCommand: "
    print serialPort.portstr
    print serialPort.isOpen()
    
    # Below is an example of mtHeadCommand message with fixed parameters #
    # setupData = [64, 48, 48, 51, 67, 60, 0, 255, 2, 55, 19, 128, 255, 1, 1, 35, 11, 102, 102, 102, 5, 102, 102, 102, 5, 112, 61, 10, 9, 112, 61, 10, 9, 40, 0, 60, 0, 128, 12, 128, 12, 210, 0, 84, 84, 125, 0, 125, 0, 25, 64, 208, 0, 144, 1, 244, 1, 100, 0, 64, 6, 1, 0, 0, 0, 10]
    
    message_flag = 0
    while(message_flag !=4) and not rospy.is_shutdown():                        # Continuously sends mtHeadCommand message until sonar responds
        setupData = get_mtHeadCommand()
        serialPort.write(setupData)
        time.sleep(0.1)
        print 'still waiting for mtAlive....'
        message_flag = sonarListener()
        
    serialPort.flushOutput()                                                    # Flush serial buffers to ensure a clean start of operation                    
    serialPort.flushInput()

    #setupData = get_mtHeadCommand()                                             # NEEDS TESTING ####### why are we writing the setup data twice...?
    #serialPort.write(setupData)


################################################################
def get_mtHeadCommand():
    # Constructs mtHeadCommand message -> see manual for more information #
    
    # hard-coded as specific to this message: #
    AsciiBlock = [64, 48, 48, 51, 67]                                           # @003C - ASCII header to msg 
    hexLength  = [60, 0]                                                        # Length of block in 2 bit int
    count      = 55 		                                                # Number of bytes following count byte - excluding LF character
    msgNum     = 19 	                                                        # Msg identification number
    
    # Construct message using parameters, see launch file #
    SID = rospy.get_param("/SID")
    DID = rospy.get_param("/DID")

    headType = rospy.get_param("/headType")

    msgSeq = rospy.get_param("/msgSeq")
    Node   = rospy.get_param("/Node")
  
    HdCtrl1 = int(rospy.get_param("/HdCtrl1"),2)                                # Important settings, see manual
    HdCtrl2 = int(rospy.get_param("/HdCtrl2"),2)                                # Important settings, see manual

    DstHead = rospy.get_param("/DstHead")
    TXNChan = [102, 102, 102, 5]                                                # Hard coded as not used by DST sonars
    RXNChan = [112, 61, 10, 9]                                                  # Hard coded as not used by DST sonars

    txPulseLength = [40, 0]                                                     # Hard coded as not used by DST sonars
    rangeScale = [60, 0]                                                        # Hard coded as not used by sonar head
   
    LLim = rospy.get_param("/LLim")                                             # Convert anti-clockwise scan limit from degrees to 16ths of a gradian 
    LLim = (LLim)%360
    LLim = LLim/360.0 * 6400

    RLim = rospy.get_param("/RLim")                                             # Convert clockwise scan limit from degrees to 16ths of a gradian
    RLim = (RLim)%360
    RLim = RLim/360.0 * 6400


    LLim = uint16_to_uint8(int(LLim))                                           # Convert from 1 x UINT16 to 2 x UINT8
    RLim = uint16_to_uint8(int(RLim))                                           # Convert from 1 x UINT16 to 2 x UINT8

    ADSpan = rospy.get_param("/ADSpan")
    ADLow  = rospy.get_param("/ADLow")

    IGainB1 = rospy.get_param("/IGainB1")
    IGainB2 = rospy.get_param("/IGainB2")

    Slope = [125, 0, 125, 0]                                                    # Hard coded as not used by DST sonars

    moTime = rospy.get_param("/moTime")
    step = rospy.get_param("/step")
    
    NBins = rospy.get_param("/NBins")
    Range = rospy.get_param("/Range")
    
    ##################################
    ADInterval = uint16_to_uint8(int(round((((Range*2)/1500.0)/NBins)/0.000000640,0))) # Possibly not using it, not actually sent to sonar and only used for post-processing
    rospy.set_param("/ADInterval", ADInterval)                                  # Updates ADInterval to parameter server for user in sonar_detect_obstacle
    ##################################
    
    NBins     = uint16_to_uint8(NBins)
    MaxADBuf  = uint16_to_uint8(rospy.get_param("/MaxADBuf"))
    Lockout   = uint16_to_uint8(rospy.get_param("/Lockout"))
    MinorAxis = uint16_to_uint8(rospy.get_param("/MinorAxis"))
    MajorAxis = rospy.get_param("/MajorAxis")

    Ctl2  = rospy.get_param("/Ctl2")
    ScanZ = [0, 0]                                                              # Hard coded as always set to 0
    LF    = 10                                                                  # Hard coded as always set to 10

    # Constructs mtHeadCommand to send to sonar #
    mtHeadCommand = AsciiBlock + hexLength + [SID, DID, count, msgNum, msgSeq, Node, headType, HdCtrl1, HdCtrl2, DstHead] + TXNChan + TXNChan + RXNChan + RXNChan + txPulseLength + rangeScale + LLim + RLim +[ADSpan, ADLow, IGainB1, IGainB2] + Slope + [moTime, step] + ADInterval+ NBins + MaxADBuf + Lockout + MinorAxis + [MajorAxis, Ctl2] + ScanZ + [LF]

    print mtHeadCommand
    try:
        mtHeadCommand = ''.join([chr(char) for char in mtHeadCommand[:]])
    except ValueError:
        print 'Error with mtHeadCommand line 231'
        
    return mtHeadCommand

################################################################
def uint16_to_uint8(input16):
    # Converts a single uint16 to two uint8s #
    B1 = int(numpy.uint8(input16))
    B2 = (input16-B1)/256
    output8 = [B1, B2]
    return output8

################################################################
def callback(data):
    # Beta-code, used to update left and right limits during sonar operation.  Not currently used and needs testing. #
    global updateFlag
    updateFlag = 1
    rospy.set_param("/LLim", int(data.LLim))
    rospy.set_param("/RLim", int(data.RLim))

################################################################
def sonarLoop():
    # Main operational loop #
    # Controls sonar message sequence (mtSendData->, ->mtHeadData, mtSendData->, ->mtHeadData,... etc.  #

    rospy.Subscriber('sonar_update', sonar, callback)                           # Only used for receiving new parameters (callback(data))
    
    global updateFlag
    global serialPort
    
    updateFlag = 0                                                              # updateFlag is set to 1 if new parameters are recieved

    setupSonar()

    while not rospy.is_shutdown():                                              # Whilst ROS is running 
        sonarTalker()                                                           # Constructs and send mtSendData message to request a ping
        sonarListener()                                                         # Waits for response then processes response
        
        if updateFlag == 1:                                                     # If new parameters have been received then reconstruct and send mtHeadCommand
            setupData = get_mtHeadCommand()
            serialPort.write(setupData)
            updateFlag = 0
        
        time.sleep(0.001)
            
################################################################

def shutdown():
    global serialPort
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.close()

################################################################
################################################################
if __name__ == '__main__':

    rospy.init_node('sonar')
    
    global pub
    global updateFlag
    
    pub        = rospy.Publisher('sonar_output', String)
    updateFlag = 0
    
    rospy.on_shutdown(shutdown)

    sonarLoop()                                                                 # Main execution loop
    
