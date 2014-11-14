#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import time

#from joy.msg import Joy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from hardware_interfaces.msg import altitude
from std_msgs.msg import Bool
from hardware_interfaces.msg import compass as compass_out

global last_depthd
global last_headingd
global last_swayd
global compass
global pub_heading_control_onOff
global altitude1

def joy_cb(data):
    global last_headingd
    global last_depthd
    global last_swayd
    global pub_heading
    global pub_depth
    global pub_speed
    global pub_sway
    global pub_tslOnOff_horiz
    global pub_tslOnOff_vert
    global pub_heading_control_onOff
    global speed_demand
    global altitude1
    
    pub_depth_and_speed_controller.publish(True)



    if data.buttons[1]:
        #change depth by +0.1 - button 2
        last_depthd = last_depthd + 0.1
        pub_depth.publish(last_depthd)
	#print 'depth demand', last_depthd
        pub_tslOnOff_vert.publish(1)
    elif data.buttons[3]:
        #change depth by -0.1 - button 4
        last_depthd = last_depthd - 0.1
        pub_depth.publish(last_depthd)
	#print 'depth demand', last_depthd
        pub_tslOnOff_vert.publish(1)

    elif data.buttons[4]:
        #decrease sway - button 5
	if last_swayd != 0:
		pub_sway.publish(0)
		last_swayd = 0
		#print 'sway set to 0'
	elif last_swayd != -2.5:
		pub_sway.publish(-2.5)
 		last_swayd = -2.5
                #print 'sway set to -2.5'
                pub_heading.publish(compass.heading)
      	pub_tslOnOff_horiz.publish(1)
	pub_heading_control_onOff.publish(1)
    elif data.buttons[5]:
        #increase sway - button 6	
	if last_swayd != 0:
		pub_sway.publish(0)
		last_swayd = 0
                #print 'sway set to 0'
	elif last_swayd != 2.5:
		pub_sway.publish(2.5)
 		last_swayd = 2.5
		#print type(compass.heading)
                pub_heading.publish(compass.heading)
		#print 'sway set to 2.5N'
      	pub_tslOnOff_horiz.publish(1)
	pub_heading_control_onOff.publish(1)
    elif data.buttons[6]:
        #change heading by -5 - button 7
        last_headingd = last_headingd - 5
        pub_heading.publish(last_headingd)
	#print 'heading demand', last_headingd
        pub_tslOnOff_horiz.publish(1)
 	pub_heading_control_onOff.publish(1)
    elif data.buttons[7]:
        #change heading by +5 - button 8
        last_headingd = last_headingd + 5
        pub_heading.publish(last_headingd)
	#print 'heading demand', last_headingd
        pub_tslOnOff_horiz.publish(1)
	pub_heading_control_onOff.publish(1)
    elif data.buttons[9]:
        #stop - button 10
        pub_speed.publish(0.0)
	last_depthd = 0
	last_headingd = 0
	last_swayd = 0
	#print 'stop'
        pub_tslOnOff_horiz.publish(0)
        pub_tslOnOff_vert.publish(0)
    
    speed_demand = data.axes[1]*0.6
    pub_speed.publish(speed_demand)

    last_headingd=(last_headingd)%360	

    str='Heading Demand = %.1fdeg, Heading = %.1fdeg, Depth Demand = %.2fm, Depth = %.2fm, Altitude = %.2fm, Speed Demand = %.2fm/s, Sway Demand =%.2f' %(last_headingd, compass.heading, last_depthd,compass.depth,altitude1,speed_demand,last_swayd)
    print str	
  
def compass_cb(data):
    global compass
    global timer
    global last_depthd
    global altitude1
    global last_headingd
    global speed_demand
    global last_swayd
    compass = data
    
    if (time.time()-timer)>0.5:
        str='Heading Demand = %.1fdeg, Heading = %.1fdeg, Depth Demand = %.2fm, Depth = %.2fm, Altitude = %.2fm, Speed Demand = %.2fm/s, Sway Demand =%.2f' %(last_headingd,compass.heading,last_depthd,compass.depth,altitude1,speed_demand,last_swayd)
        print str
        timer=time.time()	

def callback_altitude(data):
	global altitude1    
	altitude1 = data.altitude_filt


if __name__ == '__main__':
    global last_headingd
    global last_depthd
    global last_swayd
    global speed_demand
    global pub_heading
    global pub_depth
    global pub_speed
    global pub_sway
    global pub_tslOnOff_horiz
    global pub_tslOnOff_vert
    global pub_heading_control_onOff
    global pub_depth_and_speed_controller
    global timer
    global altitude1

    rospy.init_node('joystick')
    timer=time.time() 
    last_headingd = 0.0
    last_depthd = 0.0
    last_swayd = 0.0
    speed_demand=0.0   
    altitude1=0.0
    print 'waiting 10 seconds for tail to initialise'
    #sleep for initialisation of tail
    time.sleep(5)

    pub_heading = rospy.Publisher('heading_demand', Float32)
    pub_depth = rospy.Publisher('depth_demand', Float32)
    #pub_prop = rospy.Publisher('prop_demand', Int8)
    pub_speed = rospy.Publisher('speed_demand', Float32)
    pub_sway = rospy.Publisher('sway_demand', Float32)
    pub_tslOnOff_horiz = rospy.Publisher('TSL_onOff_horizontal', Bool)
    pub_tslOnOff_vert = rospy.Publisher('TSL_onOff_vertical', Bool)
    pub_heading_control_onOff=rospy.Publisher('Heading_onOFF', Bool)   
    pub_depth_and_speed_controller = rospy.Publisher('Depth_onOFF', Bool) 

    rospy.Subscriber('joy', Joy, joy_cb)
    rospy.Subscriber('compass_out', compass_out, compass_cb)
    rospy.Subscriber('altimeter_out', altitude, callback_altitude)

    compass = compass_out()
    
    
    
    #makes sure compass data has arrived	
    print 'waiting two more seconds for compass data...'
    time.sleep(2)

    #print 'joystick now in control!'
    

    
   # TSL_onOff_horiz
   # prop_demand
   # sway_demand
   # rospy.signal_shutdown
    

    rospy.spin()
    
