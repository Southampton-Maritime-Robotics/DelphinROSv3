#!/usr/bin/python
import roslib; roslib.load_manifest('DelphinROSv2')
import rospy
import time
from DelphinROSv2.msg import compass

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String

def talker():
#        pub                     = rospy.Publisher('compass_out',compass)
#        pub_heading_demand      = rospy.Publisher('heading_demand', Float32) 
#        heading_on_off          = rospy.Publisher('Heading_onOFF', Bool)
#        pub_prop                = rospy.Publisher('prop_demand',Int8)    

        pub_speedD              = rospy.Publisher('speed_demand', Float32)
        pub_depthOnOff          = rospy.Publisher('Depth_onOFF', Bool)
        pub_depthD              = rospy.Publisher('depth_demand', Float32)
        
#        pub_SMS                 = rospy.Publisher('SMS_message', String)
#        pub_camera              = rospy.Publisher('Camera', camera)

        
        
        rospy.init_node('Simulation_setpoints')
        

        
        pub_depthD.publish(0.0)
        pub_speedD.publish(0.0)
        pub_depthOnOff.publish(True)
        time.sleep(5.0)
        pub_depthD.publish(1.5)
        pub_speedD.publish(1.0)
        pub_depthOnOff.publish(True)
        time.sleep(20.0)
#        pub_depthD.publish(1.5)
#        pub_speedD.publish(0.4)
#        pub_depthOnOff.publish(True)
#        time.sleep(40.0)
#        pub_depthD.publish(1.5)
#        pub_speedD.publish(0.6)
#        pub_depthOnOff.publish(True)
#        time.sleep(40.0)
#        pub_depthD.publish(1.5)
#        pub_speedD.publish(0.8)
#        pub_depthOnOff.publish(True)
#        time.sleep(40.0)
#        pub_depthD.publish(1.5)
#        pub_speedD.publish(1.0)
#        pub_depthOnOff.publish(True)
#        time.sleep(40.0)

        
#            heading=315.0
#            pitch=-5.0
#            roll=1.0
#            temperature=15.0		
#            depth=1.0
#            m=10.0
#            mx=5
#            my=6
#            mz=7
#            a=0
#            ax=2
#            ay=5
#            az=5
#            prop = 12.0
#            demand = 50.0
#            print 'in loop'
            
            
                        
#            pub.publish(heading=heading,pitch_filt=pitch,roll=roll,temperature=temperature,depth_filt=depth,m=m,mx=mx,my=my,mz=mz,a=a,ax=ax,ay=ay,az=az)
            
  
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass




    

    
    
