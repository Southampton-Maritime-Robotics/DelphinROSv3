#!/usr/bin/python
# PI Controller for depth
# Integral term necessary to compensate for buoyancy
# P alone would result in a constant error

import roslib; roslib.load_manifest('lowlevel_controllers')
import rospy
import serial
import time
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import position
from hardware_interfaces.msg    import compass
from lowlevel_controllers.msg   import heading_control
from std_msgs.msg               import Float32
from std_msgs.msg               import Bool

#### DEFINE GLOBAL VARIABLES ####
global flag
global controller_onOff
global Ltf       # TODO: Why is this global?
global Ltr
Ltf = 0.65       # Moment arm of front thruster [metre]
Ltr = 0.59       # Moment arm of rear thruster [metre]

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def set_params():
    global DC
    
### General ###
    DC.deadzone   = 1   # deadzone of the heading error [degree]
    
### CS Controller ###
    DC.CS_Pgain       =  0.5 #0.1(error square) 0.5(error)
    DC.CS_Igain       =  0
    DC.CS_Dgain       =  0
    DC.CS_max         =  20
    DC.CS_min         = -DC.CS_max
    
### Thrust Controller ###
    DC.Thrust_Pgain =  15000
    DC.Thrust_Igain =  0.00
    DC.Thrust_Dgain =  10000
    DC.Thrust_Smax  =  500 #  1000    

################################################################################
#### CONTROL SURFACE CONTROLLER ################################################
################################################################################

def CS_controller(error, int_error, der_error):
    global DC
    
    DC.CS_Pterm      = error*DC.CS_Pgain
####    DC.CS_Iterm      = int_error*DC.CS_Igain
# TODO may incorporate a forward speed into a consideration using gain schedualing
# TODO other option: divide the gains by u^2. If the speed is less than a threshold, all gain will be set to zero

    CS_demand = DC.CS_Pterm
####    CS_demand = DC.CS_Pterm + DC.CS_Iterm  
    CS_demand  = limits(CS_demand,DC.CS_min,DC.CS_max)
    
    DC.CSt = CS_demand
    DC.CSb = CS_demand
    
#    str = "current error is %s, and current CS_demand is %s" %(error,CS_demand) 
#    rospy.loginfo(str)

    return [DC.CSt, DC.CSb]
    
################################################################################
########## THRUST CONTROLLER ###################################################
################################################################################

def thrust_controller(error, int_error, der_error):
    global DC
    
    DC.Thrust_Pterm = error*DC.Thrust_Pgain
    DC.Thrust_Iterm = int_error*DC.Thrust_Igain
    DC.Thrust_Dterm = der_error*DC.Thrust_Dgain
    
    DC.Thrust_heading = DC.Thrust_Pterm + DC.Thrust_Iterm + DC.Thrust_Dterm
    
    ## turn torque into thrust and superposition with sway demand
    thruster0 = float(DC.Thrust_heading)/float(Ltf) + float(DC.sway)
    thruster1 = -float(DC.Thrust_heading)/float(Ltr) + float(DC.sway)
    
    if numpy.abs(error) > DC.deadzone:
        DC.thruster0 = int(numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5) # according to a relationship between thrust and rpm
        DC.thruster1 = int(numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5) # according to a relationship between thrust and rpm
        # if a setpoint of one thruster goes beyond the limit. it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        if numpy.abs(DC.thruster0) > DC.Thrust_Smax:
            scale_factor = float(DC.Thrust_Smax)/float(numpy.abs(DC.thruster0))
            DC.thruster0 = int(DC.thruster0*scale_factor)
            DC.thruster1 = int(DC.thruster1*scale_factor)
        if numpy.abs(DC.thruster1) > DC.Thrust_Smax:
            scale_factor = float(DC.Thrust_Smax)/float(numpy.abs(DC.thruster1))
            DC.thruster0 = int(DC.thruster0*scale_factor) 
            DC.thruster1 = int(DC.thruster1*scale_factor)
    else:
        DC.thruster0 = 0
        DC.thruster1 = 0
    
    str = ">>>>>>>>>>>>>>>>Heading demand is %.2fdeg" %((DC.heading_demand)%360) 
    rospy.loginfo(str)  
    str = ">>>>>>>>>>>>>>>>Current heading demand is %.2fdeg" %(DC.heading) 
    rospy.loginfo(str)
    str = ">>>>>>>>>>>>>>>>Error is %.2fdeg" %(error) 
    rospy.loginfo(str)
    str = ">>>>>>>>>>>>>>>>Thruster0 setpoint demand is %d" %(DC.thruster0) 
    rospy.loginfo(str)
    str = ">>>>>>>>>>>>>>>>Thruster1 setpoint demand is %d" %(DC.thruster1) 
    rospy.loginfo(str)
    print ''
        
    return [DC.thruster0, DC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global flag
        global controller_onOff
        global speed
        global DC
        
        DC = heading_control()
        flag             = False
        speed            = 0
        controller_onOff = Bool()
        delta_t          = 0.1
        set_params()
                
        time_zero        = time.time()        
        [error, int_error, der_error] = system_state(-1)                            # On first loop, initialize relevant parameters
        
        while not rospy.is_shutdown():

            dt = time.time() - time_zero                                            # Calculate the elapse time since last calculation
#            if dt >= delta_t:        
            if dt >= delta_t and controller_onOff == True:

                time_zero = time.time()
                # Get system state #
                [error, int_error, der_error] = system_state(dt)
                # Control Surface Controller # Nb CSp = Sternplane port, CSt = Rudder top
                [CSt, CSb] = CS_controller(error, int_error, der_error)
                    
                # Thruster controller # 
                [thruster0, thruster1] = thrust_controller(error, int_error, der_error)
                
####                print 'CSt = ',CSt
####                print 'T2 = ',thruster0
####                print 'T3 = ',thruster1

                # update the heading_control.msg, and this will be subscribed by the logger.py
                pub_tail.publish(cs0 =CSt, cs1 = CSb)
                pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
                pub_DC.publish(DC) 
            else:
                time.sleep(0.01)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def system_state(dt):
    global DC
    global int_error
    global der_array_size
    global sample
    global cumulative_time

### ERROR ###
    demand = (DC.heading_demand)%360
    error  = demand - DC.heading
    
    if error <-180:
        error =   error%360
    elif error > 180:
        error= -(-error%360)            

    if dt == -1:
        sample = numpy.zeros(2)
        int_error = 0
        der_error = 0
####        der_array_size = 100
####        sample = numpy.zeros(der_array_size)
####        cumulative_time = numpy.zeros(der_array_size)
               
    else:

### INTEGRAL ###
        int_error += dt*error					# Calculate the integral error       

### DERIVATIVE ###
        # this simple calculation is good enough for the xsens
        sample[1] = sample[0]	                # Shift old values up in the array
        sample[0] = error				        # Set first array term to new error value
        der_error = (sample[0]-sample[1])/dt    # Calculate the integral error
####        sample[1:der_array_size] = sample[0:(der_array_size-1)]	# Shift old values up in the array
####        sample[0] = error				                # Set first array term to new depth value

####        cumulative_time[1:der_array_size] = cumulative_time[0:(der_array_size-1)]	# Shift old values up in the array
####        cumulative_time[0] = time_zero	
####        
####        coeffs = numpy.polyfit(cumulative_time, sample, 1)
####        der_error = DC.der_error = coeffs[0]
        
    # update the error terms. These will be subscribed by the logger node.
    DC.error = error
    DC.int_error = int_error
    DC.der_error = der_error

    return [error, int_error, der_error]

################################################################################
######## SATURATION AND UPDATE PARAMETERS FROM TOPICS ##########################
################################################################################

def limits(value, min, max):       #Function to contrain within defined limits
    if value < min:				   
       value = min
    elif value > max:
       value = max
    return value

################################################################################

def heading_demand_cb(headingd):
    global flag
    global DC
    DC.heading_demand = headingd.data
    flag = True   
    
def sway_demand_cb(swaydemand):
    global DC
    DC.sway = swaydemand.data
    flag = True

def compass_cb(compass):
    global flag
    global DC
    DC.heading = compass.heading
    flag = True

def onOff_cb(onOff):
    global controller_onOff
    controller_onOff=onOff.data
    
def speed_callback(data):
    global speed
    speed = data.forward_vel
    DC.speed = speed
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Heading_controller')
    
    global DC
    DC = heading_control()
   
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('sway_demand', Float32, sway_demand_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('position_dead', position, speed_callback)
    rospy.Subscriber('Heading_onOFF', Bool, onOff_cb)
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
    pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
    pub_DC   = rospy.Publisher('Heading_controller_values', heading_control)
    
    rospy.loginfo("Heading controller online")

    main_control_loop()
