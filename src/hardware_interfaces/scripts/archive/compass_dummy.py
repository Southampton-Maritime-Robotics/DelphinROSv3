#!/usr/bin/python
import roslib; roslib.load_manifest('hardware_interfaces')
import rospy
import time
from hardware_interfaces.msg import compass
from hardware_interfaces.msg import position
from hardware_interfaces.msg import camera

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String

def talker():
        pub                     = rospy.Publisher('compass_out',compass)
#        pub_heading_demand      = rospy.Publisher('heading_demand', Float32) 
#        heading_on_off          = rospy.Publisher('Heading_onOFF', Bool)
#        speed                   = rospy.Publisher('position_dead', position)
        pub_prop                = rospy.Publisher('prop_demand',Int8)    

        pub_speedD              = rospy.Publisher('speed_demand', Float32)
        pub_depthOnOff          = rospy.Publisher('Depth_onOFF', Bool)
        pub_depthD              = rospy.Publisher('depth_demand', Float32)
        
        pub_SMS                 = rospy.Publisher('SMS_message', String)
        pub_camera              = rospy.Publisher('Camera', camera)

        
        
        rospy.init_node('compass_dummy')
        pub_depthOnOff.publish('True')
        
        #time.sleep(1.0)
        
        #pub_SMS.publish('Testing Delphin2 SMS node!!!!')
        
#        pub_camera.publish(1, True, 'cam1.avi')
#        time.sleep(10.0)
#        pub_camera.publish(0, False, 'cam1.avi')

        while not rospy.is_shutdown():
            heading=315.0
            pitch=-5.0
            roll=1.0
            temperature=15.0		
            depth=1.0
            m=10.0
            mx=5
            my=6
            mz=7
            a=0
            ax=2
            ay=5
            az=5
            prop = 12.0
            demand = 50.0
            print 'in loop'
            
            
                        
            pub.publish(heading=heading,pitch=pitch,roll=roll,temperature=temperature,depth=depth,m=m,mx=mx,my=my,mz=mz,a=a,ax=ax,ay=ay,az=az)
            pub_depthD.publish(2)
            pub_speedD.publish(1)
            pub_depthOnOff.publish(True)
            rospy.sleep(0.50)
  
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass




    

    
    
