#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import serial
import time
import numpy
import math
import string
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tsl_feedback
from hardware_interfaces.msg import status

from std_msgs.msg import UInt8
from std_msgs.msg import Bool

global new_data
global current_data
global serialPort
global max_setpoints
global onOff

############################# INITIALISE SERIAL ######################################    
def init_serial():
    global serialPort
    global current_data
    current_data = {'thruster0':0,'thruster1':0,'thruster2':0,'thruster3':0}
    
    serialPort = serial.Serial(port='/dev/ttyS2', baudrate='57600', timeout=0.1)
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print serialPort.isOpen()
    return serialPort.isOpen()

############################# INITIALISE BOARD ######################################    
def init_board():
    global serialPort
    global max_setpoints
    
    try:
        max_setpoints = [rospy.get_param("/thruster0-setpoint-max"),rospy.get_param("/thruster1-setpoint-max"),rospy.get_param("/thruster2-setpoint-max"),rospy.get_param("/thruster3-setpoint-max")]
    except:
        max_setpoints = [2500,2500,2500,2500]
    
    print max_setpoints
    
    setPoints = ''
    # make string from the initial data
    setPoints = ' %d %d %d %d 0 0\n' %(current_data['thruster0'],current_data['thruster1'],current_data['thruster2'],current_data['thruster3'])
    print setPoints
    dataToSend = 'p' + setPoints # add the block write character
    serialPort.flushInput() # flush to be safe
    print 'inside init_board'
    motor_shutdown() # Shut down motors just in case.
    serialPort.write(dataToSend) # send setpoints
    serialPort.write('w 0 259\n') # Turning off watchdog!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    time.sleep(0.2)
    voltage = 0
    
    voltage = tsl_getVoltage() #Gets voltage data from board to check that board is sending and recieving messages
    
    if voltage == 0:
        return False
    else:
        return True
    

############################# GET VOLTAGE ######################################    

def tsl_getVoltage():
    serialPort.flushInput()   
    serialPort.write('R 8 \n')                                              #Request motor voltage Nb. this is not the same as battery voltage
        
    while serialPort.inWaiting() < 6:                                       #Wait until there is serial available
        time.sleep(0.001)
                
    while serialPort.inWaiting() > 0 and serialPort.read(1) == 'A':         #While there is data to be read - read the line...
        voltage = int(serialPort.readline())	        #Read in line of data
        print 'Voltage Data: ', voltage
        
    return voltage

############################# MAIN LOOP ############################################    
def tsl_cust(status):
    global onOff_horiz
    global onOff_vert
    onOff_horiz = 0
    onOff_vert = 0
    time_zero = time.time()	

    
    while not rospy.is_shutdown():            
        
        pubStatus.publish(nodeID = 1, status = status)
        
        data_now = current_data                                                 #Get the data at the current time
        
        ############################# ON/OFF MSG ######################################    
        thrust_sp = [data_now['thruster0']*onOff_vert, data_now['thruster1']*onOff_vert, data_now['thruster2']*onOff_horiz, data_now['thruster3']*onOff_horiz]
        non_zero_indexes = numpy.flatnonzero(numpy.array(thrust_sp) != 0)
        sum = 0
        
        for i in non_zero_indexes:
            sum += pow(2, i)
        
        #print 'w 0 %d' %(sum)
        serialPort.write('w 0 %d\n' %(sum))
    
        ############################# SATURATION ######################################    
        for i in range(len(thrust_sp)):
            if (abs(thrust_sp[i])>max_setpoints[i]):
                thrust_sp[i] = math.copysign(max_setpoints[i], thrust_sp[i])    #Set the setpoint to the maximum value if it exceeds it, retaining the sign.
        
        
        ############################# SETPOINT MSG ####################################    

        setpoint0=-thrust_sp[0]                                                 #*onOff_vert#.vert_thrusters_on
        setpoint1=-thrust_sp[1]                                                 #*onOff_vert#.vert_thrusters_on 
        setpoint2= thrust_sp[2]                                                 #*onOff_horiz#.horiz_thrusters_on 
        setpoint3= thrust_sp[3]                                                 #*onOff_horiz#.horiz_thrusters_on 

        setPoints = ' %d %d %d %d 0 0\n' %(setpoint0, setpoint1, setpoint2, setpoint3)  #Create setpoint message
        
        ############################# SETPOINTS ######################################    
        print 'Sending setpoints'
        serialPort.write('p 24' + setPoints)                                    #Send thruster setpoints
                
        while serialPort.inWaiting() < 20:                                      #Wait until there is serial available
            time.sleep(0.001)
        
        time.sleep(0.025)                                                       #Delay necessary to enable TSL board to achieve request
        serialPort.flushInput()                                                 #Flush data back from setpoints
        
        ############################### RPM ##########################################
        serialPort.write('G 32 \n')                                             #Request thruster speeds
        
        while serialPort.inWaiting() < 15:
            time.sleep(0.001)
             
        while serialPort.inWaiting() > 0 and serialPort.read(1) == 'A':         #While there is data to be read - read the line...
            speed_data = serialPort.readline()		#Read in line of data
            split_speed_data = string.split(speed_data,' ')			#Split message by comma separator
            speed0 = int(split_speed_data[1])
            speed1 = int(split_speed_data[2])
            speed2 = int(split_speed_data[3])
            speed3 = int(split_speed_data[4])
            print 'Speed Data: ', speed0, speed1, speed2, speed3

        serialPort.flushInput()                                                 #Flush to be safe

        ################################# CURRENT ####################################
        serialPort.write('G 40 \n')                                             #Request electrical current data
        
        while serialPort.inWaiting() < 15:                                      #Wait until there is serial available
            time.sleep(0.001)
               
        while serialPort.inWaiting() > 0 and serialPort.read(1) == 'A':         #While there is data to be read - read the line...
            Icurrent_data = serialPort.readline()	#Read in line of data
            split_Icurrent_data = string.split(Icurrent_data,' ')
            current0 = int(split_Icurrent_data[1])
            current1 = int(split_Icurrent_data[2])
            current2 = int(split_Icurrent_data[3])
            current3 = int(split_Icurrent_data[4])		                #Split message by comma separator
            print 'Current Data: ', current0, current1, current2, current3
        serialPort.flushInput()                                                 #Flush to be safe
    
    
        voltage = tsl_getVoltage()
        ################################# VOLTAGE ####################################
        #serialPort.write('R 8 \n')                                              #Request motor voltage Nb. this is not the same as battery voltage
        
       # while serialPort.inWaiting() < 6:                                       #Wait until there is serial available
        #    pass
                
        #while serialPort.inWaiting() > 0 and serialPort.read(1) == 'A':         #While there is data to be read - read the line...
         #   voltage = int(serialPort.readline(size = None, eol = '\n'))	        #Read in line of data
          #  print 'Voltage Data: ', voltage                                     #Print data
            
        ################################# VOLTAGE ####################################
        pub.publish(setpoint0 = setpoint0, setpoint1 = setpoint1, setpoint2 = setpoint2, setpoint3 = setpoint3, 
            speed0 = speed0, speed1 = speed1, speed2 = speed2, speed3 = speed3, 
            current0 = current0, current1 = current1, current2 = current2, current3 = current3, 
            voltage = voltage)

            
############################# CALLBACKS ######################################    

def vert_callback(new_sp):
    global current_data
    current_data['thruster0']= new_sp.thruster0
    current_data['thruster1'] = new_sp.thruster1

def horiz_callback(new_sp):
    global current_data
    current_data['thruster2'] = new_sp.thruster0
    current_data['thruster3'] = new_sp.thruster1

def motor_shutdown():
    global serialPort
    shutdown_ack = [65, 32, 48, 13, 10]
    serialPort.flushOutput()
    serialPort.write('w 0 0\n')
    time.sleep(0.001) # LIKELY TO CAUSE HANGING
    in_ = serialPort.read(5)
    msg = []
    
    for char in in_:
        msg.append(ord(char))
    
    while not msg == shutdown_ack or serialPort.inWaiting() != 0:
        msg = []

        serialPort.write('w 0 0\n') # kill all motors

        in_ = serialPort.read(5)
        for char in in_:
            msg.append(ord(char))
        print msg
        serialPort.flushInput()

def shutdown():
    motor_shutdown()
    pubStatus.publish(nodeID = 1, status = False)
    serialPort.close()

def onOff_horiz_callback(new_onOff_horiz):
    global onOff_horiz
    onOff_horiz = new_onOff_horiz.data
    print onOff_horiz
    
def onOff_vert_callback(new_onOff_vert):
    global onOff_vert
    onOff_vert = new_onOff_vert.data
    print onOff_vert

############################# START ######################################    

if __name__ == '__main__':

    time.sleep(1) #Allow System to come Online    
    
    global current_data
    rospy.init_node('TSL_customer')
    global pub
    global pubStatus
    
    
    port_status = init_serial()
    str = "TSL serial port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    
    rospy.Subscriber('TSL_onOff_horizontal', Bool, onOff_horiz_callback)
    rospy.Subscriber('TSL_onOff_vertical', Bool, onOff_vert_callback)
    rospy.Subscriber('TSL_setpoints_horizontal', tsl_setpoints, horiz_callback)
    rospy.Subscriber('TSL_setpoints_vertical', tsl_setpoints, vert_callback)
    pub = rospy.Publisher('TSL_feedback', tsl_feedback)
    pubStatus = rospy.Publisher('status', status)

    while not rospy.is_shutdown():
        status = True
        pubStatus.publish(nodeID = 1, status = status)
        rospy.loginfo("TSL board now online")

    board_status = init_board()
    
    
    if board_status and port_status == True:
        status = True
        pubStatus.publish(nodeID = 1, status = status)
        rospy.loginfo("TSL board now online")
    else:
        status = False
        pubStatus.publish(nodeID = 1, status = status)
        str ="TSL board not online"
        rospy.loginfo("TSL board not online")
    
    rospy.on_shutdown(shutdown)
    tsl_cust(status)
