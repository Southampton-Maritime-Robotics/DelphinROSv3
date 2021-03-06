"""
VERSION AS OF 2016-04-08
Library for interfacing with the tritech sonar via the serial protocol
for details on the protocol, see datasheets

Messages are assembled as individual array entries in uint8 format
by entering either an interger or hex value e.g. 64 or 0x40
for sending via serial, they need to be converted to chr 

Implemented messages:
- mtAlive: broadcast by sonar head at 1-second time interval after centering and arming
- mtReBoot: reboots sonar head; currently used before every setting change
   Note: this will slow down the setting changes!
- mtHeadCommand: sends a new set of settings to the sonar head
- mtSendData: request scan from sonar head
- mtHeadData: This one is taken apart in sonar_analyse, after receiving it here


TODO: Make the reboot for every setting change optional
"""

import numpy
import rospy
import time
import serial


class SonarTritech:
    """
    Basic class for controlling the mechanical scanning sonar:
    - reset and update settings
    - receive returns
    - assemble messages
    - decode messages
    
    TODO: Do something nice with the resolution of the range
    """

    def __init__(self):
        # read initial sonar settings from parameter server
        self.DriftOffset = rospy.get_param("sonar/Driftoffset")  # in degrees, to offset a drift of the scanning head
        if self.DriftOffset != 0:
            rospy.logerr("Using sonar drift offset of " + str(self.DriftOffset) + " degrees!")
        self.SoundspeedWater = float(rospy.get_param("sonar/SoundspeedWater")) # make sure this value is float
        self.RLim = rospy.get_param("sonar/RLim")  # in degrees
        self.LLim = rospy.get_param("sonar/LLim")  # in degrees
        self.__dict__['NBins'] = rospy.get_param("sonar/NBins")  # avoid setattr error, since range is not set yet
        self.Range = rospy.get_param("sonar/Range")
        self.ADInterval = int(round((((self.Range * 2) / 1500.0) / self.NBins) / 0.000000640, 0))
        self.Resolution = self.Range/float(self.NBins) # currently unused resolution of range, in [m]
        self.ADSpan = rospy.get_param("sonar/ADSpan")
        self.ADLow = rospy.get_param("sonar/ADLow")
        self.IGainB1 = rospy.get_param("sonar/IGainB1")
        self.IGainB2 = rospy.get_param("sonar/IGainB2")
        self.MoTime = rospy.get_param("sonar/MoTime")
        self.Step = rospy.get_param("sonar/Step")
        self.Data = []  # char array, where the recorded sonar ping data goes
        self.lastMtAlive = []
        self.updateFlag = 0
        self.SerialParam = rospy.get_param("sonar/serial")  # A dictionary of the various serial components
        self.receiveErrorCount = 0  # catch when communication with the sonar gets out of sync
        self.serial_setup()
        self.update_sonar_head_setting()

    def __setattr__(self, key, value):
        """
        A) Apply drift offset automatically
        B) Update ADInterval when  NBins or Range are set
        
        A) In 2017 the sonar incurred a large amount of drift
        This drift *should* get removed if the sonar is reset, however just in case this becomes a problem
        during an experiment when there is no time to fix, this value offset can work as a hotfix
        It shifts 0 degrees to the DriftOffset angle
        
        B)
        ADInterval is defined indirectly through NBins and Range, so if either changes, it needs updating
        R = Range, N = NBins, VOS = Sound velocity in water
        T = 2*R / VOS; # in [s]
        t = T/N; # in [s]
        ADInterval = t/(640*10**-9); in [640 nanoseconds]
        """

        # A) DRIFTOFFSET
        if key == "RLim":
            value = (value - self.DriftOffset)%360

        if key == "LLim":
            value = (value - self.DriftOffset)%360

        # B) ADINTERVAL
        if key == "Range":
            self.ADInterval = int(round((((value * 2) / self.SoundspeedWater) / self.NBins) / 0.000000640, 0))
            self.Resolution = value/float(self.NBins)

        if key == "NBins":
            if value > 1500:
                value = 1500
            self.ADInterval = int(round((((self.Range * 2) / self.SoundspeedWater) / value) / 0.000000640, 0))
            self.Resolution = self.Range/float(value)

        if key == "ADInterval":
            if value > (256**2 - 1):
                value = 256**2 - 1
        self.__dict__[key] = value

    def update_sonar_head_setting(self):
        """
        Reboot sonar head in case drift occurred
        Update the header settings to the current settings via the mtHeadCommand
        typically the mtHeadCommand can be sent separately, but is necessary after sending the mtReboot command
        however, with issues of the sonar head drifting, the reset is always performed
        stepper motor position and head info are monitored
        """
        rospy.logwarn("updated sonar tritech settings")
        mtReBoot = self.get_mtReBoot()
        self.Serial.write(uint8_array_to_chr(mtReBoot, 'mtReBoot'))

        # wait for mtAlive
        message_type = 0
        while (message_type != 4) and not rospy.is_shutdown():
            #rospy.logdebug("Waiting for mtAlive after sending mtReBoot")
            # wait a little longer for mtAlive message from transducer
            rospy.sleep(0.1)
            message_type = self.read_sonar()

        # Flush serial buffers to ensure clean operation
        self.Serial.flushInput()
        self.Serial.flushOutput()

        # send mtHeadCommand message
        # wait for type 4 message mtAlive:
        self.updateFlag = 0
        mtHeadCommand = self.get_mtHeadCommand()
        self.Serial.write(uint8_array_to_chr(mtHeadCommand, 'mtHeadCommand'))

        start_t = time.time()
        message_type = 0
        while (message_type != 4) and (time.time() - start_t < 60) and not rospy.is_shutdown():
            #rospy.loginfo("Waiting for mtAlive after sending mtHeadCommand")
            if time.time() - start_t > 50:
                rospy.logwarn("waiting for mtAlive after sending mtHeadCommand - check mtHeadCommand"
                              + str(mtHeadCommand))
            rospy.sleep(0.1)
            message_type = self.read_sonar()

        # Flush serial buffers to ensure clean operation
        self.Serial.flushInput()
        self.Serial.flushOutput()
        self.send_ping_trigger() # maintain write-ahead: send first ping trigger

    def get_mtReBoot(self):
        return [0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x10, 0x80, 0x02, 0x0A]

    def get_mtHeadCommand(self):
        """
        construct mtHeadCommand message for updating the sonar head setting
        setting values for 
        * LLim, RLim [given  in degrees, transmitted in 1/16th gradians]
        * MoTime [given and transmitted in 10 microseconds
        * Step [Step, given and transmitted in 1/16th gradians]
        * NBins [number of bins, up to 1500]
        * Range *indirectly*: ADInterval is automatically updated based on Range and NBins
        
        TODO: 
        * A future interesting value is the HdCtrl1 and HdCtrl2 setting,
          which together change the sonar head configuration
        * ADSpan and ADLow can be used to remap the dynamic range, but only in 4 bit mode. 
          We are currently opperating in 8 bit mode, so it seems this doesn't do anything
        
        notes on the content:
        - single channel DST, so extra 16 byte appendage is neglected, message length is 60
        - total length of message is 66
            -> the HexLength from byte 6 onwards, excluding LF is 66 - 6 = 60
            -> the byte count No. Byte after the No. Byte entry is 60 - 5 = 55
        - Tx Node and Rx Node are as in documentation, but might change in combination with other devices
        - mtHeadCommandID is 19, ir 0x13 in hex
        - ignored parameters were set to 0 for simplicity, before they were:
                TXNChan = [102, 102, 102, 5]
                RXNChan = [112, 61, 10, 9]
                txPulseLength = [40,0]
                Slope = [125, 0, 125, 0]
        """

        HdCtrl1 = int(self.SerialParam['HdCtrl1'], 2)
        HdCtrl2 = int(self.SerialParam['HdCtrl2'], 2)
        llim = (self.LLim * 6400/360.) % 6400  # convert from degrees to 1/16th Gradians, make sure value is below 6400
        rlim = (self.RLim * 6400/360.) % 6400
        # check the Step size is within the standard settings
        #if not (self.Step in [4, 8, 16, 32]):
            #rospy.logwarn("Setting sonar step size to non-standard value " + str(self.Step))

        mtHeadCommand = (
            # Hdr, Hex Length, Bin Length                                  {1| 2, 3, 4, 5 | 6 7}
            [0x40] + int_to_hex_length_uint8_array(60, 4) + [60, 0] +
            # Tx Nde - serial ID, Rx Nde - device ID                       { 8 | 9}
            [self.SerialParam['SID'], self.SerialParam['DID']] +
            # No. Byte, mtHeadCommand ID 19, Message Sequence Bitset=End   {10 | 11 | 12}
            [55] + [0x13] + [0x80] +
            # repeat of Tx Nde, V3B Params                                 {13 | 14}
            [self.SerialParam['SID'], self.SerialParam['HeadType']] +
            # HdCtrl, HdType                                               {15, 16 | 17}
            [HdCtrl1, HdCtrl2, self.SerialParam['DstHead']] +
            # TXN Ch1,Ch2, RXN Ch1, Ch2, Tx PulseLen ignored by DST        {18 to 21|22 to 25|26 to 29|30 to 33|34,35}
            [0] * 18 +
            # range (ignored), LLim, RLim                                  {36, 37| 38, 39| 40, 41}
            number_to_uint8(self.Range, 2) + number_to_uint8(llim, 2) + number_to_uint8(rlim, 2) +
            # ADSpan, ADLow,                                               {42 | 43}
            number_to_uint8(self.ADSpan, 1) + number_to_uint8(self.ADLow, 1) +
            # Igain Ch1, Igain Ch2                                         {44 | 45}
            number_to_uint8(self.IGainB1, 1) + number_to_uint8(self.IGainB2, 1) +
            # Slope Ch1, Ch2 ignored by DST                                {46, 47| 48, 49}
            [0] * 4 +
            # MoTime, Step Angle Size                                      {50 | 51}
            number_to_uint8(self.MoTime, 1) + number_to_uint8(self.Step, 1) +
            # ADInterval, NBins                                            {52, 53| 54, 55}
            number_to_uint8(self.ADInterval, 2) + number_to_uint8(self.NBins, 2) +
            # MaxADBuf, Lockout                                            {56, 57| 58, 59}
            number_to_uint8(self.SerialParam['MaxADBuf'], 2) + number_to_uint8(self.SerialParam['Lockout'], 2) +
            # Minor Axis Direction, Major Axis Pan                         {60, 61| 62}
            number_to_uint8(self.SerialParam['MinorAxis'], 2) + number_to_uint8(self.SerialParam['MajorAxis'], 1) +
            # Ctl2, ScanZ, LF                                              {63| 64, 65| 66}
            [0x00] + [0x00, 0x00] + [0x0A]
        )
        #rospy.logdebug("Assembled new mtHeadCommand: " +
        #               str(mtHeadCommand[0:14]) + "\n" +
        #               str(mtHeadCommand[14:28]) + "\n" +
        #               str(mtHeadCommand[28:42]) + "\n" +
        #               str(mtHeadCommand[42:56]) + "\n" +
        #               str(mtHeadCommand[56:])   + "\n")

        return mtHeadCommand

    def get_mtSendData(self):
        """
        Assemble the mtSendData message to request a sample set
        - total length of message is 18 bytes
            -> the HexLength from byte 6 onwards, excluding LF is 18 - 6 = 12
            -> the byte count No. Byte after the No. Byte entry is 12 - 5 = 7
        
        :return: uint8 array for mtSendData 
        """

        mtSendData = (
            # Hdr, HexLength, BinLength                                         {1|2, 3, 4, 5| 6, 7}
            [0x40] + int_to_hex_length_uint8_array(12, 4) + number_to_uint8(12, 2) +
            # Tx Nde - serial ID, Rx Nde - device ID                            { 8| 9}
            [self.SerialParam['SID'], self.SerialParam['DID']] +
            # No. Byte, mtSendData ID 25, Message Sequence Bitset=End           {10| 11| 12}
            [7] + [0x19] + [0x80] +
            # repeat of Tx Nde, current Time fixed at zero (not needed), LF     {13| 14, 15, 16, 17| 18}
            [self.SerialParam['SID']] + [0] * 4 + [0x0A]
            )
        return mtSendData

    def send_ping_trigger(self):
        # assemble array of mtHeadCommand serial communication message
        mtSendData = self.get_mtSendData()

        # send mtSendData message to sonar
        self.Serial.write(uint8_array_to_chr(mtSendData, "mtSendData"))
        #rospy.logdebug("Sending ping trigger to sonar, mtSendData Message: \n"
        #               + str(mtSendData[0:13]) + "\n"
        #               + str(mtSendData[13:]))

    def read_sonar(self):
        """
        Read ping reply from the sonar as headerData = byte 1 to 13 and msgData = byte 14 to end
        determine message type,
        update sonar data stored in self (if applicable to message type)
        """
        startTime = time.time()
        while self.Serial.inWaiting() <= 12:
            # wait until number of bytes is the length of the message header
            time.sleep(0.001)
            if startTime + self.SerialParam['ReadTimeout'] < time.time():
                self.Serial.flushInput()
                rospy.logwarn("Sonar serial: read header timeout")
                self.serial_handle_error()
                return 0
        if self.Serial.inWaiting() > 12:
            # header Data was received,
            # analyse header data and receive to remaining message length
            headerData = self.Serial.read(13)
            headerData = numpy.fromstring(headerData, dtype=numpy.uint8)
            #rospy.logdebug("Data from sonar: mtHeadData: "
            #               + str(headerData))

            if headerData[0] == 64:
                # every message should start with and '@' (ASCII 64)
                # communication is still working, so receive error count is reset
                self.receiveErrorCount = 0
                (msgType, msgLength) = analyse_header(headerData)
                #rospy.logdebug("Message type: " + str(msgType) +
                #               " \n Message length: " + str(msgLength))
                startTime = time.time()
                while self.Serial.inWaiting() < msgLength:
                    time.sleep(0.001)
                    if startTime + self.SerialParam['ReadTimeout'] < time.time():
                        rospy.logwarn("Sonar serial: read message timeout")
                        self.Serial.flushInput()
                        self.serial_handle_error()
                        return 0

                msgData = self.Serial.read(self.Serial.inWaiting())
                msgData = numpy.fromstring(msgData, dtype=numpy.uint8)
                #rospy.logdebug("Data from sonar: message data: "
                #               + str(msgData))

                if msgData[-1] != 10:
                    #rospy.logdebug("Message error: Message does not end in '\\LF'")
                    self.Serial.flushInput()
                    return 0

                if msgType == 1:
                    #rospy.logdebug("Message type: mtVersionData \n"
                    #               + "Message data: " + msgData)
                    return 1

                elif msgType == 2:
                    msgData = numpy.concatenate((headerData, msgData))
                    # convert from numpy array into string so it can be published
                    self.Data = uint8_array_to_chr(msgData, 'mtHeadData')

                    # debug calculations
                    transBearing = msgData[27] + (msgData[28] * 256)
                    transBearing = (transBearing / 6400.00) * 360
                    #rospy.logdebug("Message type: mtHeadData\n"
                    #               "TransBearing: " + str(transBearing))

                    return 2

                elif msgType == 4:
                    #rospy.logdebug("Message type: mtAlive")
                    self.lastMtAlive = msgData
                    self.analyse_mtAlive()    # Analysis step, not needed for operation of sonar
                    return 4

                else:
                    #rospy.logdebug("Unknown message type: " + str(msgType))
                    pass
            else:
                #rospy.logdebug("Sonar message does not start with the required '@'\n"
                #              + "Starting with: " + str(headerData[0]))
                self.serial_handle_error()

    def analyse_mtAlive(self):
        """
        content of mtAlive after header bytes {14 to 21} in the docs :
        [0]        14: 'WillSend' byte, no longer used for DST
        [1:5]   15-18: Head Time
        [5:7]   19-20: Stepper Motor Position
        [7]        21: 'HeadInf'
        [8]        22: Message Terminator
        
        note: "InCentre" means a re-centre operation is ongoing
        """
        byte = 2 ** 8
        stepperPosition = self.lastMtAlive[5] + self.lastMtAlive[6] * byte
        headInf = {}
        bitset = bin(self.lastMtAlive[7] + byte)  # add byte to make sure the bin value is full length
        headInf['InCentre'] = bitset[-1]  # byte 0 is last entry of bitset string
        headInf['Centred'] = bitset[-2]
        headInf['Motoring'] = bitset[-3]
        headInf['Motor On'] = bitset[-4]
        headInf['Dir'] = bitset[-5]
        headInf['InScan'] = bitset[-6]
        headInf['NoParams'] = bitset[-7]
        headInf['SentCfg'] = bitset[-8]
        #rospy.loginfo("Decoded mtAlive: Stepper Position is: " + str(stepperPosition) +
        #              "Header info is: " + str(headInf))

    def read_setting_demand(self, message):
        newRLim = message.RLim
        if newRLim != self.RLim:
            self.updateFlag = 1
            self.RLim = newRLim

        newLLim = message.LLim
        if newLLim != self.LLim:
            self.updateFlag = 1
            self.LLim = newLLim

        newNBins = message.NBins
        if newNBins != self.NBins:
            self.updateFlag = 1
            self.NBins = int(newNBins)

        newRange = message.Range
        if newRange != self.Range:
            self.updateFlag = 1
            self.Range = newRange

    def serial_shutdown(self):
        self.Serial.flushInput()
        self.Serial.flushOutput()
        self.Serial.close()

    def serial_setup(self):
        try:
            self.Serial = serial.Serial(port=self.SerialParam['Port'], baudrate=self.SerialParam['Baudrate'])
            self.Serial.bytesize = serial.EIGHTBITS
            self.Serial.stopbits = serial.STOPBITS_ONE
            self.Serial.parity = serial.PARITY_NONE
            #rospy.loginfo("Setup serial port for sonar: "
            #              + str(self.Serial.portstr) + '\n'
            #              + str(self.Serial.isOpen()))
        except:
            rospy.logwarn("Sonar could not be mounted to given serial port " + str(self.SerialParam['Port']))

    def serial_reset(self):
        # this function is called to restart the serial communication
        self.serial_shutdown()
        self.serial_setup()
        self.update_sonar_head_setting()

    def serial_handle_error(self):
        # Always update error count
        self.receiveErrorCount += 1
        # Flush serial buffers to ensure clean operation
        self.Serial.flushInput()
        self.Serial.flushOutput()

        # if error count is too high, reset serial communication
        if self.receiveErrorCount >= self.SerialParam['CriticalErrorCount']:
            rospy.logerr("Resetting sonar serial communication: "
                         "Error count too high, too many messages did not start with required '@'")
            self.receiveErrorCount = 0
            self.serial_reset()


def analyse_header(headerData):
    msgType = headerData[10]
    msgLength = (headerData[5] + (headerData[6] * 256)) - 7
    # get the length of the rest of the data
    # calculate from mtAlive 22 bytes in total, header[5:7] = 16 bytes, we load in 13 bytes initially
    # thus number to remove = 16 - (22-13) = 7
    # headerData[5]+headerData[6]*256  converting 2 uint8s to 1 uint16 
    return msgType, msgLength




##################################
#
# DATA CONVERSIONS
#
##################################

def number_to_uint8(number, out_len):
    """
    Converts a number to a uint8 array of given length, will give a warning if number is too large for length
    :param number: input number, will be rounded, converted to int, then split in uint8
    :param out_len: length of the output array
    :return: output array with the input array encoded in it; left is least significant
    """
    if number > (256**out_len - 1):
        rospy.logwarn("assembling sonar message: Trying to send number larger than the bytes it needs to fit in!")

    int_number = int(round(number))
    result = [0] * out_len
    while out_len > 0:
        out_len = out_len - 1
        result[out_len] = int(int_number / (256 ** out_len))  # starting with MSB, enforce division result stays int
        int_number = int_number % (256 ** out_len)  # get remainder
    return result


def uint8_array_to_chr(intArray, messageName):
    """ 
    converts int arrays e.g. assembled messages into chr for serial writing
    ([64, 0x40] -> '@@')
    """
    try:
        result = ''.join([chr(data) for data in intArray[:]])
        return result
    except:
        # This will throw an error e.g. if the int value is larger than 256
        rospy.logwarn("Error converting message from int array to char for " + messageName)


def int_to_hex_length_uint8_array(value, out_len):
    """
    converts a number into the message format 'Hex Length' that is used in the messages
    Example from mtHeadCommand Hex Length
    67 = 0 *16**3 + 0 *16**2 + 4*16 + 12 -> entry_chr are 0, 0, 4, 12
    ['0', '0', '4', 'C'] -> strings representing the hex format of entry_chr
    [48, 48, 52, 67]
    [0x30, 0x30, 0x34, 0x43]
    :param value: number to which the Hex Length entry will be set
    :param out_len: number of bytes in the message over which the number is transmitted
    :return: array of int values, length of this array is bytes
    """
    # fill the array with '0'
    result = [ord('0')] * out_len
    while out_len > 0:
        out_len = out_len - 1
        # get current byte and remainder
        entry_chr = value / 16 ** out_len  # int value of the hex character to be sent, e.g. 0 for '0', 10 for 'a'
        value = value % (16 ** out_len)
        # enter current byte in array
        # transform entry_chr in the hex format of the value
        # enter the int representation of the string representing that value
        if entry_chr < 10:
            result[-(out_len+1)] = ord('0') + entry_chr
        else:
            result[-(out_len+1)] = ord('A') + entry_chr - 10
    return result


def hex_length_uint8_array_to_int(array):
    """
    converts an int array containing a message in 'Hex Length' format into a number
    :param array: int array containing 'Hex Length' format
    :return: int of the number encoded in the int array
    """
    result = 0
    for idx, value in enumerate(array):
        # find correct power of 16
        # leftmost value has highest power
        p = len(array) - 1 - idx
        # convert the string representation of hex value into a number and apply correct power of 16
        if value <= ord('9'):
            result = result + (value - ord('0')) * 16 ** p
        else:
            result = result + (value - ord('A') + 10) * 16 ** p

    return result
