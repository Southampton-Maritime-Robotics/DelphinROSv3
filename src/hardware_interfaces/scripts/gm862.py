#!/usr/bin/python

#import roslib; roslib.load_manifest('hardware_interfaces')
#import rospy
import serial
import re
import time
import numpy
import string

################################################################
################################################################
def startup():
    ######## Check board is on ########
    send('AT\r')                     # Check system online
    #time.sleep(0.5)
    answer = read()
    
    if answer == '':
        answer = read()
    
    if answer[0] == 'AT' and answer[1] == 'OK':
        print 'SMS Working'
    else:
        print 'Error.  read returned = ',answer
        return False
    
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
            s = str(answer[1])
            strength = int(s[6:8])
            print 'Signal = ',strength
            return strength
    except:
            return 0
        
################################################################
################################################################
def sendSMS(number,message):  
    
    s = 'AT+CMGS = ' + str(number) + '\r'
    print 'cmd = ',s
    send(s)
    
    answer = read()
    
    if ('>' in answer) == True:
        print 'Typing message... '
        send(message)
        serialPort.write(chr(0x1A))
        
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
                s = (str(answer[1]))
                number = int(s[22:23])
                print 'Number of unread messages = ',number
                return number
            except:
                try:
                    s = (str(answer[1]))
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
        cmd = 'AT+CMGR = ' + str(memory) + '\r'
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
    cmd = 'AT+CMGD = ' + str(memory) + '\r'
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
