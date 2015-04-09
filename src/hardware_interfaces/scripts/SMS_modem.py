#!/usr/bin/python
import rospy
import serial
import time
import numpy
import string
from datetime import datetime
from std_msgs.msg import String
from std_msgs.msg import Bool
from DelphinROSv2.msg import status
from DelphinROSv2.msg import dead_reckoner
from DelphinROSv2.msg import tsl_feedback
from DelphinROSv2.msg import SMS

################################################################################
################################################################################
def sms_loop():
    while not rospy.is_shutdown():
        
        global number
        global time_zero
        global newmessage
        global newmessage_flag
        
        number = '"+447921926092"' #Leo
        #number = '"+447403850752"' #Alex
        #number = '"+447737502702"' #Catherine
        #number = '"+447752779315"' #Maaten
        
        time_zero = 0.0
        time_window = 30.0
        newmessage = ''
        newmessage_flag = False
        

        while not rospy.is_shutdown():
            
            if network_status() == True and signal_strength() > 1 and (time.time() - time_zero) > time_window:
                time_zero = time.time()
                
                if newmessage_flag:
                    try:
                        sendSMS(number,data.data)
                    except ValueError:
                        print 'Failed to send message'
                    time.sleep(8)
                    sendStatusSMS()
                    newmessage_flag = False
                    time.sleep(0.5)
                else:
                    sendStatusSMS()                                
                    time.sleep(0.5)
                print '########################################'
                
                
            if receivedSMS() > 0:
                print 'Message received!!!'
                [message, memory] = readSMS()
                print 'Message = ',message
                deleteSMS(memory)
                time.sleep(0.5)
            
            pubSMSinfo.publish(signal = signal_strength(), depth = DR.Z, latitude = DR.latitude, longitude = DR.longitude)
                
            time.sleep(0.5)

################################################################################

def sendStatusSMS():
    try:
        voltage  = tsl.voltage
        GPS_lat  = DR.latitude
        GPS_long = DR.longitude
        GPS_fix  = DR.ValidGPSfix
        
        stringtime = datetime.now()
        T    = stringtime.strftime('%H-%M-%S')
        date = stringtime.strftime('%d-%m-%Y')
        
        message =   'Delphin2_AUV Status: \n' + \
                    'Time: %s' %(T) + '\n' + \
                    'Date: %s' %(date) + '\n' + \
                    'Voltage: %s' %(voltage) + '\n' + \
                    'Lat: %s' %(GPS_lat) + ', ' + 'Long: %s' %(GPS_long) + '\n' + \
                    'Fix: %s' %(GPS_fix) 

        sendSMS(number,message)
        print 'Message sent'           
        done = True
        read()
        
    except ValueError:
        print 'Error sending status message'
        
################################################################################
################################################################################
def setUpSerial(): # set up the serial port
    global serialPort
    try:
        serialPort = serial.Serial(port='/dev/usbSMS', baudrate='38400', timeout=0.5) 
        serialPort.bytesize = serial.EIGHTBITS
        serialPort.stopbits = serial.STOPBITS_ONE
        serialPort.parity = serial.PARITY_NONE
        print 'Initialised SMS serial.'
        return serialPort.isOpen()
    except:
        print 'Serial port failed to open.  Check that board is connected and switch on.'

  
     
################################################################################
####################### Telit GM862 driving functions ##########################
################## See documentation folder for more info ###################### 
################################################################################

def startup():
    ######## Check board is on ########
    send('AT\r')                     # Check system online
    #time.sleep(0.5)
    answer = read()
    
    if answer == '':
        answer = read()
    
    board_on = False
    
    while not board_on:
        try:
            
            if numpy.size(answer) > 1:
                if answer[0] == 'AT' and answer[1] == 'OK':
                    print 'SMS Working'
                    board_on = True
            else:
                print 'Error.  read returned = ',answer
                
                print 'Switching on phone...'
                pubOnOff.publish(True)
                time.sleep(10)
                send('AT\r')                     # Check system online
                #time.sleep(0.5)
                answer = read()
                
                if answer == '':
                    answer = read()
                    
        except ValueError:
            'in value error loop'
            send('AT\r')                     # Check system online
            #time.sleep(0.5)
            answer = read()
            time.sleep(0.5)
    
######## Enable extended error results codes ########
    #time.sleep(1)
    send('AT+CMEE=1\r')                     # Check system online
    answer = read()
    
    if answer[0] == 'AT+CMEE=1' and answer[1] == 'OK':
        print 'Extended error result codes enabled'
    else:
        print 'Error.  read returned = ',answer
        return False

######## Query SIM presence and status ########
    #time.sleep(1)
    send('AT+CPIN?\r')                     # Check system online
    answer = read()
    
    if answer[0] == 'AT+CPIN?' and answer[1] == '+CPIN: READY' and answer[3] == 'OK':
        print 'SIM card working'
    else:
        print 'Error - problem with sim. Returned = ',answer
        return False
    
######## Set SMS format to text ########
    #time.sleep(1)
    send('AT+CMGF=1\r')                     # Check system online
    answer = read()
    
    if answer[0] == 'AT+CMGF=1' and answer[1] == 'OK':
        print 'SMS set to text'
    else:
        print 'Error. Returned = ',answer
        return False
    
    return True

################################################################
################################################################
def network_status():
    
    #time.sleep(1)
    send('AT+CREG?\r')                    # Check system online
    answer = read()
    
    try:
        if ('+CREG: 0,1' in answer):
            '+CREG: 0,1' in answer
            print 'Network available'
            return True
    except:
        answer = read()
        if ('+CREG: 0,1' in answer):
            print 'Network available'
            return True
        else:
            print 'Network unavailable'
            return False
        
################################################################
################################################################
def signal_strength():
    
    #time.sleep(1)
    send('AT+CSQ\r')                    # Check system online
    answer = read()
    try:
        if ('OK' in answer):
            s = '%s' %answer[1]
            strength = int(s[6:8])
            print 'Signal = ',strength
            return strength
    except:
        return 0
        
################################################################
################################################################
def sendSMS(number,message):  
    
    s = 'AT+CMGS = %s' %(number) + '\r'
    print 'cmd = ',s
    send(s)
    
    answer = read()
    
    if ('>' in answer) == True:
        print 'Typing message... '
        send(message)
        serialPort.write(chr(0x1A))
    
    print 'Message = ',message
        
################################################################
################################################################
def receivedSMS():  
    number = 0
    allowed_attempts = 3
    attempts = 0
    print 'Checking messages'
    
    while attempts < allowed_attempts:
        send('AT+CPMS?\r')
        answer = read()
        
        if ('AT+CPMS?' in answer) and ('OK' in answer):
            try:
                s = '%s' %(answer[1])
                number = int(s[22:23])
                print 'Number of unread messages = ',number
                return number
            except:
                try:
                    s = '%s' %(answer[1])
                    number = int(s[22:24])
                    print 'Number of unread messages = ',number
                    return number
                except:
                    pass
                
        attempts = attempts + 1
    return number
    
################################################################
################################################################
def readSMS():  
    
    memory = 1
    while True:
        cmd = 'AT+CMGR = %s' %(memory) + '\r'
        send(cmd) 
        
        answer = read()
        
        if not '+CMS ERROR: 321' in answer:
            try:
                message = process_message(answer)
                return [message, memory]
            except:
                pass
            
        if memory == 20:
            print 'no messages found!'
            return 
        
        memory = memory + 1

################################################################
################################################################
def deleteSMS(memory):  
    
    print 'Deleting message in memory position = ',memory
    cmd = 'AT+CMGD = %s' %(memory) + '\r'
    send(cmd)
    
    answer = read()

    if 'OK' in answer:
        print 'message deleted'
        return True
    else:
        print 'failed to delete message'
        return False
    
################################################################
################################################################
def process_message(data):
    
    i = 0
    while i < 10:
        if data[i] == 'OK':
            break
        i = i + 1
        
    message = data[2:i-1]
    return message
    
################################################################
################################################################
def send(message):      
    serialPort.write(message)

################################################################
################################################################
def read():
    timeout = 5
    time_zero = time.time()
    i = 0
    lists = [[]] * 50
    
    while serialPort.inWaiting() <= 0 and time.time() - time_zero < timeout:
        pass
        
    while serialPort.inWaiting() > 0:
        try: 
            data = serialPort.readline(size = None, eol = '\n')
            time.sleep(0.01)
        except:
            pass
            #print 'Failed to read line'
        lists[i] = data.rstrip()
        i = i + 1
    
    DATA = lists[0:i]
    
    #print DATA
    try:
        return DATA
    except:
        return 

################################################################
################################################################

################################################################################
def shutdown():
    
    send('AT#SHDN\r')
    time.sleep(2.0)
    
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 6, status = False)
    serialPort.close()
    
################################################################################
def DR_callback(data):
    global DR
    DR = data
    
################################################################################
def tsl_callback(data):
    global tsl
    tsl = data    

################################################################################ 
def new_message_callback(data):
    global time_zero
    global newmessage
    global newmessage_flag
    
    newmessage = data.data
    newmessage_flag = True
    
    time_zero = time.time()
        
################################################################################
################################################################################
################################################################################
if __name__ == '__main__':
    time.sleep(5) #Allow System to come Online    
        
    rospy.init_node('SMS_modem')
    rospy.on_shutdown(shutdown)
    
    global pubStatus
    global DR
    global tsl
    DR = dead_reckoner()
    tsl = tsl_feedback()
    
    ###########################################
    #### Set up subscribers and publishers ####
    rospy.Subscriber('dead_reckoner', dead_reckoner, DR_callback)
    rospy.Subscriber('TSL_feedback', tsl_feedback, tsl_callback)        
    rospy.Subscriber('SMS_message', String, new_message_callback)        

    pubStatus  = rospy.Publisher('status', status, queue_size=3)
    pubSMSinfo = rospy.Publisher('SMS_info', SMS, queue_size=3)
    pubOnOff   = rospy.Publisher('phone_onOff', Bool, queue_size=3)
    time.sleep(0.5)
    
    print 'Switching on phone...'
    pubOnOff.publish(True)
    time.sleep(10)
    
    ######################
    #### Setup serial ####
    print 'Setting up serial...'
    
    serial_status = setUpSerial()
    if serial_status:
        print 'Serial initialised.\n'
    else:
        serial_status = setUpSerial()
        if status:
            print 'Serial initialised.\n'
        else:
            print 'Serial failed to be initialised.'
            
    string = "SMS modem serial port status = %s. Port = %s" %(serial_status, serialPort.portstr)
    
    
    
    ########################
    #### Start up board ####
    startup_status = startup()
    
    if (serial_status and startup_status) == True:    
        pubStatus.publish(nodeID = 6, status = True)
        rospy.loginfo("SMS modem now online")
    else:
        pubStatus.publish(nodeID = 6, status = False)
    
    print '########################################'
    
    #########################
    #### Start main loop ####
    rospy.on_shutdown(shutdown)
    sms_loop()
