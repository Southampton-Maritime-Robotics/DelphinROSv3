#!/usr/bin/python
import roslib; roslib.load_manifest('lowlevel_controllers')
import rospy
import serial
import time
import numpy
from hardware_interfaces.msg import tsl_setpoints
from hardware_interfaces.msg import tail_setpoints
from hardware_interfaces.msg import position
from hardware_interfaces.msg import compass
from lowlevel_controllers.msg import heading_control
from std_msgs.msg import Float32
from std_msgs.msg import Bool

#### DEFINE GLOBAL VARIABLES ####
global flag
global cur_depthd
global cur_compass
global controller_onOff
global cur_headingd

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def set_params():
    global HC
    global CS_INT_ERROR
    
    CS_INT_ERROR = 0.0
    
### General ###
    HC.min_int_error  =-5
    HC.max_int_error  = 5
    HC.aft_deadzone   = 2
    HC.fwd_deadzone   = 5
    
### CS Controller ###
    HC.CS_Pgain       =  0.1 #0.5
    HC.CS_min         = -20 
    HC.CS_max         =  20
    
### Thrust Controller ###
    HC.Thrust_Pgain =  0.05 #0.01
    HC.Thrust_Igain =  0.00
    HC.Thrust_Dgain =  0.05 # 0.005
    HC.Thrust_Pmin  = -16
    HC.Thrust_Pmax  =  16
    HC.Thrust_Imin  = -2.5
    HC.Thrust_Imax  =  2.5
    HC.Thrust_Dmin  = -5
    HC.Thrust_Dmax  =  5
    HC.Thrust_Smin  =-1200 # -1000
    HC.Thrust_Smax  = 1200#  1000    

################################################################################
#### CONTROL SURFACE CONTROLLER ################################################
################################################################################

def CS_controller(error, int_error, der_error, cur_compass):
    global HC
    global CS_INT_ERROR

### CS Controller ###
    
    HC.CS_Pterm      = (abs(error)* error*0.25 * HC.CS_Pgain)
    #CS_INT_ERROR     = limits((CS_INT_ERROR + error*0.1),-10,10)
    
    #CS_Iterm = CS_INT_ERROR * 0.5
    print 'CS_Pterm = ',HC.CS_Pterm
    
#    if speed > 0.5:
#        CS_demand    = limits((HC.CS_Pterm)/(speed*abs(speed)),HC.CS_min,HC.CS_max)
#    else:
#        CS_demand    = limits((HC.CS_Pterm)/(0.25),HC.CS_min,HC.CS_max)
        
    CS_demand  = limits((HC.CS_Pterm),HC.CS_min,HC.CS_max)
    
    HC.CSt = CS_demand
    HC.CSb = CS_demand

    return [HC.CSt, HC.CSb]
    
################################################################################
########## THRUST CONTROLLER ###################################################
################################################################################

def thrust_controller(error, int_error, der_error):
    

    HC.Thrust_Pterm = limits(error*HC.Thrust_Pgain, HC.Thrust_Pmin, HC.Thrust_Pmax)
#    HC.Thrust_Pterm = limits(abs(error)*error*HC.Thrust_Pgain, HC.Thrust_Pmin, HC.Thrust_Pmax)

    HC.Thrust_Iterm = limits(int_error*HC.Thrust_Igain, HC.Thrust_Imin, HC.Thrust_Imax)
    HC.Thrust_Dterm = limits(der_error*HC.Thrust_Dgain, HC.Thrust_Dmin, HC.Thrust_Dmax)
    
    if not numpy.isnan(HC.Thrust_Dterm):
        HC.Thrust_heading = HC.Thrust_Pterm + HC.Thrust_Iterm + HC.Thrust_Dterm
    else:
        HC.Thrust_heading = HC.Thrust_Pterm + HC.Thrust_Iterm
            
    return [HC.Thrust_heading]

################################################################################
########## INVERSE MODEL + SWAY ################################################
################################################################################

def inverse_model(Thrust_heading, error):

    thruster0 = 0.95*(Thrust_heading + float(HC.sway))
    thruster1 = 1.05*(-Thrust_heading + float(HC.sway))
    
    if numpy.abs(error) > HC.fwd_deadzone:
        HC.thruster0 = int(limits((numpy.sign(thruster0)*(60*(numpy.abs(thruster0)/(1000*0.46*0.07**4))**0.5)), HC.Thrust_Smin, HC.Thrust_Smax))
    else:
        HC.thruster0 = 0
        
    if numpy.abs(error) > HC.aft_deadzone:
        HC.thruster1 = int(limits((numpy.sign(thruster1)*(60*(numpy.abs(thruster1)/(1000*0.46*0.07**4))**0.5)), HC.Thrust_Smin, HC.Thrust_Smax))
    else:
        HC.thruster1 = 0
        
    return [HC.thruster0, HC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global flag
        global cur_depthd
        global cur_compass
        global controller_onOff
        global speed
        global HC
        
        HC = heading_control()
        flag             = False
        cur_heading      = Float32()
        cur_compass      = compass()
        speed            = 0
        controller_onOff = Bool()
        delta_t          = 0.1
        time_zero        = time.time()
        set_params()
        
        print 'About to enter main loop'
        
        while not rospy.is_shutdown():

            dt = time.time() - time_zero                                            # Calculate time since last calculation
        
            if dt >= delta_t and controller_onOff == True:
                    
                    time_zero = time.time()
                    # Get system state #
                    [error, int_error, der_error] = system_state(cur_heading, cur_compass, HC.min_int_error, HC.max_int_error)
                    
                    # Control Surface Controller # Nb CSp = Sternplane port, CSt = Rudder top
                    [CSt, CSb] = CS_controller(error, int_error, der_error, cur_compass)
                        
                    # Thruster controller # 
                    [Thrust_heading] = thrust_controller(error, int_error, der_error)
                    
                    [thruster0, thruster1] = inverse_model(Thrust_heading, error)
                    
                    print 'CSt = ',CSt
                    print 'T2 = ',thruster0
                    print 'T3 = ',thruster1

                    #pub_tail.publish(cs0 =CSt, cs1 = CSb)
                    pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
                    pub_HC.publish(HC)
            else:
                    time.sleep(0.01)
                    
                    #print HC

################################################################################
######## END OF LOW LEVEL CONTROLLER ###########################################
################################################################################


################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def system_state(cur_headingd, cur_compass, min_int_error, max_int_error):
    global time_zero
    global int_error
    global der_array_size
    global sample
    global cumulative_time
    global HC
    
    try:
        dt = time.time() - time_zero				# Calculate dt time between last loop interation
        time_zero = time.time()
    except:
        time_zero = time.time()                                 # On first loop define time zero and int_error
        dt = time.time() - time_zero
        int_error = 0
        der_array_size = 100
        sample = numpy.zeros(der_array_size)
        cumulative_time = numpy.zeros(der_array_size)

### ERROR ###

    demand = (HC.heading_demand)%360

    error  = demand - HC.heading
    
    if error <-180:
        error =   error%360
    if error > 180:
        error= -(-error%360)

    HC.error = error
    
### INTEGRAL ###   
    int_error += dt*error					# Calculate the integral error       
    int_error = HC.int_error = limits(int_error,HC.min_int_error,HC.max_int_error)

### DERIVATIVE ###
    sample[1:der_array_size] = sample[0:(der_array_size-1)]	# Shift old values up in the array
    sample[0] = error				                # Set first array term to new depth value

    cumulative_time[1:der_array_size] = cumulative_time[0:(der_array_size-1)]	# Shift old values up in the array
    cumulative_time[0] = time_zero	
    
    coeffs = numpy.polyfit(cumulative_time, sample, 1)
    der_error = HC.der_error = coeffs[0]
    
    return [error, int_error, der_error]
################################################################################
################################################################################

################################################################################
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
    global cur_headingd
    global HC

    cur_headingd = headingd
    HC.heading_demand = headingd.data
    flag = True   
    
def sway_demand_cb(swaydemand):
    global HC
    HC.sway = swaydemand.data
    flag = True

def compass_cb(compass):
    global flag
    global cur_compass
    global HC

    cur_compass = compass 
    HC.heading = compass.heading
    flag = True

def onOff_cb(onOff):
    global controller_onOff
    controller_onOff=onOff.data
    
def speed_callback(data):
    global speed
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
