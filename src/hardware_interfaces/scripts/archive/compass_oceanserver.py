

#!/usr/bin/python
import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import numpy
import serial
import time
import math
from re import findall
from DelphinROSv2.msg import compass
from DelphinROSv2.msg import status
global serialPort


################################################################
def setUpSerial():
    global serialPort
    serialPort = serial.Serial(port='/dev/usboceanserver', baudrate='115200', timeout=0.01) # may need to change timeout if having issues!
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    print "Initialised OceanServer serial."
    print serialPort.portstr
    print serialPort.isOpen()
    serialPort.flushInput()
    serialPort.flushOutput()
    return serialPort.isOpen()
    
    
################################################################
def listenForData(status):
    global serialPort
    
    time_zero = time.time()
  
    #### FILTER INFO ####
    depth_array_length = 20
    Dx = numpy.zeros([depth_array_length],float)
    Dy = numpy.zeros([depth_array_length],float)
    for i in range(0,depth_array_length): 
        Dx[i] = i
            
    
    pitch_array_length = 10
    Px = numpy.zeros([pitch_array_length],float)
    Py = numpy.zeros([pitch_array_length],float)
    for i in range(0,pitch_array_length): 
        Px[i] = i
        
    depth_old = 0.0
    
    #####################   
    
    
    while not rospy.is_shutdown():    
        try:
            time.sleep(0.01)  # Prevents node from using 100% CPU!!
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$':     #while there is data to be read - read the line...
                
                pubStatus.publish(nodeID = 5, status = status)
                
                dataRaw = serialPort.readline(size = None, eol = '\n')
                                
                data = numpy.array((findall('[-]*\d+.\d+',dataRaw)), numpy.float)
                
                try:

                    dt = time.time() - time_zero
                    print dt
                    time_zero = time.time()
                    
                    
                    pitch_raw   = data[2]
                    
                    pitch       = pitch_raw-180
                    if pitch <-180:
                        pitch=pitch%360
                        
                    #pitch = pitch_raw
                    
                    roll_raw    = data[1]    
                    roll        =-roll_raw     
                    
                    #roll = roll_raw
                              
                    temperature = data[3]          
                    depth       = (data[4]*42.045)- 0.84+0.53  -0.21  -0.05   #Convert ADC value to depth value
                    
#                    if (abs(depth - depth_old)) > 1.0:
#                        depth = depth_old
                                        
                    m           = data[5]
                    mx          = data[7]
                    my          =-data[6]  
                    mz          =-data[8]  
                    
                    mx_raw      = data[6]
                    my_raw      = data[7]
                    mz_raw      = data[8]
                    
                    mx = mx_raw
                    my = my_raw
                    mz = mz_raw
                    
                                        
                    heading     = calibrate(mx_raw,my_raw,mz_raw, pitch_raw, roll_raw)
                    
                    a           = data[9]
                    ax          = data[11]
                    ay          =-data[10]
                    az          =-data[12]
                                        
                    #### DEPTH FILTER ####
                    Dx_real = Dx * dt
                    
                    Dy[1:depth_array_length] = Dy[0:(depth_array_length-1)]
                    Dy[0] = depth
                    [der, depth_filt] = numpy.polyfit(Dx_real, Dy, 1)
                    depth_der = -der
                    
                    #depth_filt = depth
                    
                    
                    #### PITCH FILTER ####
                    Px_real = Px * dt
                    Py[1:pitch_array_length] = Py[0:(pitch_array_length-1)]
                    Py[0] = pitch
                    [der, pitch_filt] = numpy.polyfit(Px_real, Py, 1)
                    pitch_der = -der
                    
                                        
                    print '*******'
                    print 'heading %f' %(heading)
#                    print 'pitch %f' %(pitch)                    
                    print 'pitch (filtered) %f' %(pitch)
#                    print 'pitch_der (filtered) %f' %(pitch_der)
                    print 'roll %f' %(roll)
                    print 'temperature %f' %(temperature)
#                    print 'depth %f' %(depth)
                    print 'depth (filtered) %f' %(depth)
#                    print 'depth_der (filtered) %f' %(depth_der)
#                    print 'm %f' %(m)
#                    print 'mx %f' %(mx)
#                    print 'my %f' %(my)
#                    print 'mz %f' %(mz)
#                    print 'a %f' %(a)
#                    print 'ax %f' %(ax)
#                    print 'ay %f' %(ay)
#                    print 'az %f' %(az)
     
                    #Publish data to compass_out
                    pub.publish(heading=heading,pitch=pitch,roll=roll,temperature=temperature,depth=depth,m=m,mx=mx,my=my,mz=mz,a=a,ax=ax,ay=ay,az=az,depth_filt=depth_filt,depth_der=depth_der,pitch_filt=pitch_filt,pitch_der=pitch_der)
                    
                except ValueError: 
                    print 'not a float'
                
        except:
            print 'read error'
        
        #print 'sleeping'
        

################################################################

def calibrate(mx,my,mz, pitch, roll):

    pitch = numpy.radians(roll)
    roll  = numpy.radians(pitch)

        
    U = numpy.matrix([[0.00583711864398940, 0.000619779486949836, 9.27558703674774e-06],[0, 0.00478543996712305, -2.33933064978229e-06],[0, 0, 6.05590876286348e-05]])
    
    c = numpy.matrix([[-51.8129873735954],[-27.4232376953900],[13686.8142270094]])

    
    v = numpy.matrix([[mx],[my],[mz]])
    
    w = numpy.dot(U,(v - c))  #Normalised m terms
    
    eqn22_topline = (w[2]*numpy.sin(roll) - w[1]*numpy.cos(roll))
    
    eqn22_botline = (w[0]*numpy.cos(pitch) + w[1]*numpy.sin(pitch)*numpy.sin(roll) + w[2]*numpy.sin(pitch)*numpy.cos(roll))
    
    heading2 = math.atan2(eqn22_topline,eqn22_botline)*180/numpy.pi
    
    heading3 = (-heading2 + 90)%360
    
    
    return heading3
    
    
################################################################

def validDataCheck():
    attempts = 1
    
    while attempts < 5:
        
        while not serialPort.read(1) == '$':
            pass
        dataRaw = serialPort.readline(size = None, eol = '\n')		#Read in line of data
        data = findall('[-]*\d+.\d+',dataRaw)
        print data
        if len(data) == 13:
            return True
    
    return False
################################################################
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 5, status = False)
    serialPort.close()

################################################################        
#     INITIALISE     ###########################################
################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online    
    rospy.init_node('OceanServer_compass')
    
    global pub
    global serialPort
    
    pub = rospy.Publisher('compass_out', compass)   
    pubStatus = rospy.Publisher('status', status)
    
    rospy.on_shutdown(shutdown)  
    
    port_status = setUpSerial()
    str = "OceanServer port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    time.sleep(0.3)
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:    
        status = True
        pubStatus.publish(nodeID = 5, status = status)
        rospy.loginfo("OceanServer online")
    else:
        status = False
        pubStatus.publish(nodeID = 5, status = status)
      
    
    listenForData(status)   #Main loop for receiving data
    
    
