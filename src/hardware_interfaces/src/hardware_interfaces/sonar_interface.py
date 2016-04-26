import numpy
import rospy
import time
import serial

"""
VERSION AS OF 2016-04-08
"""

class SonarTritech:
    """
    Basic class for controlling the mechanical scanning sonar,
    updating its setting
    and receiving its returns
    """
    def __init__(self):
        # read initial sonar settings from parameter server
        self.RLim = rospy.get_param("sonar/RLim")
        self.LLim = rospy.get_param("sonar/LLim")
        self.NBins = rospy.get_param("sonar/NBins")
        self.ADInterval = rospy.get_param("sonar/ADInterval")
        self.ADSpan = rospy.get_param("sonar/ADSpan")
        self.ADLow = rospy.get_param("sonar/ADLow")
        self.IGainB1 = rospy.get_param("sonar/IGainB1")
        self.IGainB2 = rospy.get_param("sonar/IGainB2")
        self.MoTime = rospy.get_param("sonar/MoTime")
        self.Step = rospy.get_param("sonar/Step")
        self.SerialParam = rospy.get_param("sonar/serial") # A dictionary of the various serial components
        try:
            self.Serial = serial.Serial(port=self.SerialParam['Port'], baudrate = self.SerialParam['Baudrate'])
        except:
            rospy.logerror("Sonar could not be mounted to given serial port " + str(self.SerialParam))
        self.Serial.bytesize = serial.EIGHTBITS
        self.Serial.stopbits = serial.STOPBITS_ONE
        self.Serial.parity = serial.PARITY_NONE
        rospy.logdebug("Setup serial port for sonar: " 
                        + str(self.Serial.portstr) + '\n' 
                        + str(self.Serial.isOpen()))


        # Alternative expressions for calculating some of the above sonar head settings
        self.Range = rospy.get_param("sonar/Range")
        self.Data = [] # char array, where the recorded sonar ping data goes
        self.updateFlag = 0

        self.update_sonar_head_setting()


        
    def update_sonar_head_setting(self):
        """
        Update the header settings to the current settings via the
        mtHeadCommand
        """
        # continuously send mtHeadCommand message
        # until sonar node responds with message type 4 message mtAlive:
        self.updateFlag = 0
        message_type = 0
        while (message_type != 4) and not rospy.is_shutdown():
            mtHeadCommand = self.get_mtHeadCommand()
            self.serial.write(mtHeadCommand)
            rospy.sleep(0.1)    # 0.1 second delay  (Sophia: from old sonar code - WHY?)
            rospy.loginfo("Changing sonar head settings: Waiting for mtAlive")
            message_flag = self.read_sonar()

        # Flush serial buffers to ensure clean operation
        self.Serial.flushInput()
        self.Serial.flushOutput()

    def get_mtHeadCommand(self):
        """
        construct mtHeadCommand message for updating the sonar head setting
        """
        # TODO ??? clean up: these parameters should come from the parameter file
        # just like for the ping trigger message
        AsciiBlock = [64, 48, 48, 51, 67]
        hexLength = [60,0]
        count = 55
        msgNum = 19
        TXNChan = [102, 102, 102, 5]
        RXNChan = [112, 61, 10, 9]
        txPulseLength = [40,0]
        rangeScale=[60,0]
        Slope = [125, 0, 125, 0]
        HdCtrl1 = int(self.HdCtrl1, 2)
        HdCtrl2 = int(self.HdCtrl2, 2)
        NBins = uint16_to_uint8(self.NBins)
        MaxADBuf = uint16_to_uint8(self.MaxADBuf)
        Lockout = uint16_to_uint8(self.Lockout)
        MinorAxis = uint16_to_uint8(self.MinorAxis)
        MajorAxis = uint16_to_uint8(self.MajorAxis)
        ScanZ = [0]
        LF = 10
        mtHeadCommand = ( AsciiBlock + hexLength +
                          [ self.SID,
                            self.DID,
                            count,
                            msgNum,
                            self.MsgSeq,
                            self.Node,
                            self.HeadType,
                            HdCtrl1,
                            HdCtrl2,
                            self.DstHead] +
                         TXNChan +
                         TXNChan +    #??? Is this really using the same value twice, or am I misinterpreting this? (Sophia, 2016)
                         RXNChan +
                         RXNChan +
                         txPulseLength +
                         rangeScale +
                         LLim +
                         RLim +
                         [  self.ADSpan,
                            self.ADLow,
                            self.IGainB1,
                            self.IGainB2] +
                         Slope +
                         [  self.MoTime +
                            self.Step] +
                         self.ADInterval +
                         NBins +
                         MaxADBuf +
                         Lockout +
                         MinorAxis +
                         [MajorAxis, self.Ctl2] +
                         ScanZ + [LF])
        rospy.logdebug("mtHeadCommand: "+ mtHeadCommand)
        try:
            mtHeadCommand = ''.join([chr(char) for char in mtHeadCommand[:]])
        except ValueError:
            rospy.logerr("Error while generating the mtHeadCommand, ValueError")
        return mtHeadCommand



    def send_ping_trigger(self):
        # assemble array of mtHeadCommand serial communication message
        mtSendData = (  self.Serial['HeadeMtSendData'] + 
                        self.Serial['HexLength'] +
                        [   self.Serial['SID'],
                            self.Serial['DID'],
                            self.Serial['Count'],
                            self.Serial['MsgNum'],
                            self.Serial['MsgSeq'],
                            self.Serial['Node'] ] +
                        self.Serial['CurrentTime'] +
                        [ self.Serial['LF'] ] )
        # convert from string format to hex format
        mtSendData = ''.join([chr(char) for char in mtSendData[:]])
        # send mtSendData message to sonar
        self.Serial.write(mtSendData)
        rospy.logdebug("Sending ping trigger to sonar, mtSendData Message: \n"
                        + str(mtSendData))

    def read_sonar(self):
        """
        Read ping reply from the sonar,
        determine message type,
        update sonar data stored in self (if applicaple to message type)
        """
        startTime = time.time()
        while self.Serial.inWaiting() <= 12:
            # wait until number of bytes is the length of the message header
            time.sleep(0.001)
            if startTime + self.SerialParam['ReadTimeout'] < time.time():
                self.Serial.flushInput()
                rospy.loginfo("Sonar serial: read header timeout")
                return 0
        if self.Serial.inWaiting() > 12:
            # header Data was received,
            # analyse header data and receive to remaining message length
            headerData = self.Serial.read(13)
            headerData = numpy.fromstring(headerData, dtype=numpy.unit8)
            rospy.logdebug("Data from sonar: mtHeadData: "
                            + str(headerData))

            if headerData[0] == 64:
                # every message should start with and '@' (ASCII 64)
                (msgType, msgLength) = analyse_header(headerData)
                rospy.logdebug("Message type: " + str(msgType) +
                               " \n Message length: " + str(msgLength))
                startTime = time.time()
                while self.Serial.inWaiting() < msgLength:
                    time.sleep(0.001)
                    if startTime + self.SerialParam['ReadTimeout'] < time.time():
                        rospy.loginfo("Sonar serial: read message rimeout")
                        self.Serial.flushInput()
                        return 00

                msgData = self.Serial.read(self.Serial.inWaiting())
                msgData = numpy.fromstring(msgData, dtype = numpy.uint8)
                rospy.logdebug("Data from sonar: message data: "
                                + str(msgData))

                if msgData[-1] != 10:
                    rospy.logdebug("Message error: Message does not end in '\\LF'")
                    self.Serial.flushInput()
                    return 0

                if msgType == 1:
                    rospy.logdebug("Message type: mtVersionData \n"
                                    + "Message data: " + msgData)
                    return 1

                elif msgType == 2:
                    msgData = numpy.concatenate((headerData,msgData))
                    # convert from numpy array into string so it can be published
                    self.Data = ''.join([chr(char) for char in msgData[:]])

                    #debug calculations
                    transBearing = msgData[27] + (msgData[28]*256)
                    transBearing = (transBearing/6400.00) * 360
                    rospy.logdebug("Message type: mtHeadData\n"
                                    "TransBearing: " + str(transBearing))

                    return 2

                elif msgType == 4:
                    rospy.logdebug("Message type: mtAlive")
                    return 4

                else:
                    rospy.logdebug("Unknown message type: " + str(msgType))
            else:
                rospy.logerror("Sonar message does not start with the required '@'\n"
                               + "Starting with: "+ str(headerData[0]))

def read_setting_demand(self):
            newRLim = message.RLim
            if (newRLim != self.p['RLim']):
                self.updateFlag = 1 
                self.p['RLim'] = newRLim

            newLLim = message.LLim
            if (newLLim != self.p['LLim']):
                self.updateFlag = 1 
                self.p['LLim'] = newLLim


            newNBins = message.NBins
            if (newNBins != self.p['NBins']):
                self.updateFlag = 1 
                self.p['NBins'] = int(newNBins)

            newRange = message.Range
            if (newRange != self.p['Range']):
                self.updateFlag = 1 
                self.p['Range'] = newRange

def shutdown(self):
    self.Serial.flushInput()
    self.Serial.flushOutput()
    self.Serial.close()

 

def analyse_header(headerData):
    msgType = headerData[10]
    msgLength = (headerData[5] + (headerData[6] * 256)) - 7
    # get the length of the rest of the data
    # calulate from mtAlive 22 bytes in total, header[5:7] = 16 bytes, we load in 13 bytes initially
    # thus number to remove = 16 - (22-13) = 7
    # headerData[5]+headerData[6]*256  converting 2 uint8s to 1 uint16 
    return msgType, msgLength

def uint16_to_uint8(input16):
    # Convert a single uint16 to two uint8
    B1 = int(numpy.uint8(input16))
    B2 = (input16-B1)/256
    output = [B1, B2]
    return output8
