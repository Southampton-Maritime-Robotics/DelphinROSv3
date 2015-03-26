#!/usr/bin/python
# update: 23 October 2014 by Kantapon
# - In "CS_controller", incorporate a constant into HC.CS_Pgain
# - In "CS_controller", remove "cur_compass" from an input argument
# - In "thrust_controller", remove a seperate "limit" for P, I and D term
# - In "inverse_model", take a Moment arm of front thruster into a consideration before superposition with sway_demand
# - In "inverse_model", apply a saturation to a thruster setpoint and scale other setpoint down correspondingly.
# - if the thruster setpoint is within [0-10) of a maximum of 2300, the setpoint will be forced to 10.
# - in "system_state", remove the filter of the devirative term as the measurement took by xsens is amazingly smooth
# - simplify the structure of the heading_control.msg. this will affects set_params and logger.py

# TODO - make the "CS_controller" less agressive at high forward speeds
# TODO - check if the sway force distribution have been done correctly

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

# determine relative arm lengths for thrust allocation
global Ltf
global Ltr
L_th = 1.24         # distance between two horizontal thrusters [metre]: measured
cr = 1.05            # center of rotation on horizontal plane from the AUV nose [metre]: trial-and-error
Ltf_nose = 0.205    # location of the front horizontal thruster from nose: measured
Ltf = cr-Ltf_nose   # Moment arm of front horizontal thruster from the cr [metre]
Ltr = L_th-Ltf      # Moment arm of rear horizontal thruster from the cr [metre]

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def set_params():
    global HC
    
### General ###
    HC.deadzone   = 1   # deadzone of the heading error [degree]
    
### CS Controller ###
    HC.CS_Pgain       = 1 #0.1(error square) 0.5(error)
    HC.CS_Igain       = 0
    HC.CS_Dgain       = 0
    HC.CS_max         = 30
    HC.CS_min         = -HC.CS_max
    
### Thrust Controller ###
    HC.Thrust_Pgain = 50000.00
    HC.Thrust_Igain = 0.00
    HC.Thrust_Dgain = 30000.00
    HC.Thrust_Smax  = 800 # 1000 # maximum thruster setpoint

################################################################################
#### CONTROL SURFACE CONTROLLER ################################################
################################################################################

def CS_controller(error, int_error, der_error):
    global HC
    
    HC.CS_Pterm      = error*HC.CS_Pgain
####    HC.CS_Iterm      = int_error*HC.CS_Igain
# TODO may incorporate a forward speed into a consideration using gain schedualing
# TODO other option: divide the gains by u^2. If the speed is less than a threshold, all gain will be set to zero

    CS_demand = HC.CS_Pterm
####    CS_demand = HC.CS_Pterm + HC.CS_Iterm  
    CS_demand  = limits(CS_demand,HC.CS_min,HC.CS_max)
    
    HC.CSt = CS_demand
    HC.CSb = CS_demand
    
#    str = "current error is %s, and current CS_demand is %s" %(error,CS_demand) 
#    rospy.loginfo(str)

    return [HC.CSt, HC.CSb]
    
################################################################################
########## THRUST CONTROLLER ###################################################
################################################################################

def thrust_controller(error, int_error, der_error):
    global HC
    
    if numpy.abs(error) > HC.deadzone:
    
        HC.Thrust_Pterm = error*HC.Thrust_Pgain
        HC.Thrust_Iterm = int_error*HC.Thrust_Igain
        HC.Thrust_Dterm = der_error*HC.Thrust_Dgain
        
        HC.Thrust_heading = HC.Thrust_Pterm + HC.Thrust_Iterm + HC.Thrust_Dterm
        
        ## turn torque into thrust and superposition with sway demand
        thruster0 = float(HC.Thrust_heading)/float(Ltf) + float(HC.sway)
        thruster1 = -float(HC.Thrust_heading)/float(Ltr) + float(HC.sway)    
    
        HC.thruster0 = int(numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5) # according to a relationship between thrust and rpm
        HC.thruster1 = int(numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5) # according to a relationship between thrust and rpm
        # if a setpoint of one thruster goes beyond the limit. it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        if numpy.abs(HC.thruster0) > HC.Thrust_Smax:
            scale_factor = float(HC.Thrust_Smax)/float(numpy.abs(HC.thruster0))
            HC.thruster0 = int(HC.thruster0*scale_factor)
            HC.thruster1 = int(HC.thruster1*scale_factor)
        if numpy.abs(HC.thruster1) > HC.Thrust_Smax:
            scale_factor = float(HC.Thrust_Smax)/float(numpy.abs(HC.thruster1))
            HC.thruster0 = int(HC.thruster0*scale_factor) 
            HC.thruster1 = int(HC.thruster1*scale_factor)
    else:
    
        HC.thruster0 = 0
        HC.thruster1 = 0
        
    return [HC.thruster0, HC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global flag
        global controller_onOff
        global speed
        global HC
        
        flag             = False
        speed            = 0
        controller_onOff = Bool()
        delta_t          = 0.1
        set_params()
                
        time_zero        = time.time()        
        [error, int_error, der_error] = system_state(-1,HC.heading,(HC.heading_demand)%360)                            # On first loop, initialize relevant parameters
        
        while not rospy.is_shutdown():

            dt = time.time() - time_zero                                            # Calculate the elapse time since last calculation
            
            if dt >= delta_t and controller_onOff == True:

                time_zero = time.time()
                
                # get sampling
                heading_current = HC.heading
                heading_demand = (HC.heading_demand)%360
                
                # Get system state #
                [error, int_error, der_error] = system_state(dt,heading_current,heading_demand)
                # Control Surface Controller # Nb CSp = Sternplane port, CSt = Rudder top
                [CSt, CSb] = CS_controller(error, int_error, der_error)
                    
                # Thruster controller # 
                [thruster0, thruster1] = thrust_controller(error, int_error, der_error)
                
                # update the heading_control.msg, and this will be subscribed by the logger.py
                pub_tail.publish(cs0 =CSt, cs1 = CSb)
                pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
                pub_HC.publish(HC)
                
                self.__controller.switchHeadingOnOff(0)
                
                # verbose activity in thrust_controller
####                str = ">>>>>>>>>>>>>>>>Heading demand is %.2fdeg" %(heading_demand) 
####                rospy.loginfo(str)  
####                str = ">>>>>>>>>>>>>>>>Current heading is %.2fdeg" %(heading_current) 
####                rospy.loginfo(str)
####                str = ">>>>>>>>>>>>>>>>Heading error is %.2fdeg" %(error) 
####                rospy.loginfo(str)
####                str = ">>>>>>>>>>>>>>>>Thruster0 setpoint demand is %d" %(thruster0) 
####                rospy.loginfo(str)
####                str = ">>>>>>>>>>>>>>>>Thruster1 setpoint demand is %d" %(thruster1) 
####                rospy.loginfo(str)
####                print ''

            else:
                time.sleep(0.01)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def system_state(dt,heading_current,heading_demand):
    global HC
    global int_error
    global sample

### ERROR ###
    error = heading_demand - heading_current
    
    if error <-180:
        error =   error%360
    elif error > 180:
        error= -(-error%360)            

    if dt == -1:
        sample = numpy.zeros(2)
        int_error = 0
        der_error = 0
               
    else:

### INTEGRAL ###
        int_error += dt*error					# Calculate the integral error       

### DERIVATIVE ###
        # this simple derivative is good enough for the xsens

        # PID strategy (compute the derivative of heading error)
#        sample[1] = sample[0]	                # Shift old values up in the array
#        sample[0] = error				        # Set first array term to new error value
#        der_error = (sample[0]-sample[1])/dt    # Calculate the derivative error

        # PI-D strategy (compute the derivative of current heading)
        sample[1] = sample[0]	                # Shift old values up in the array
        sample[0] = heading_current  		        # Set first array term to new error value
        der_error  = sample[0]-sample[1]
        # dealing with a singularity
        if der_error <-180:
            der_error =   der_error%360
        elif der_error > 180:
            der_error= -(-der_error%360)
        der_error = -der_error/dt    # Calculate the derivative heading
        
    # update the error terms. These will be subscribed by the logger node.
    HC.error = error
    HC.int_error = int_error
    HC.der_error = der_error

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
    global HC
    HC.heading_demand = headingd.data
    flag = True   
    
def sway_demand_cb(swaydemand):
    global flag
    global HC
    HC.sway = swaydemand.data
    flag = True

def compass_cb(compass):
    global flag
    global HC
    HC.heading = compass.heading
    flag = True

def onOff_cb(onOff):
    global controller_onOff
    controller_onOff=onOff.data
    
def speed_callback(data):
    global speed
    global HC
    speed = data.forward_vel
    HC.speed = speed
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Heading_controller')
    
    global HC
    HC = heading_control()
   
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('sway_demand', Float32, sway_demand_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('position_dead', position, speed_callback)
    rospy.Subscriber('Heading_onOFF', Bool, onOff_cb)
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints)
    pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints)
    pub_HC   = rospy.Publisher('Heading_controller_values', heading_control)
    
    rospy.loginfo("Heading controller online")

    main_control_loop()
