#!/usr/bin/python

# import roslib; roslib.load_manifest('lowlevel_controllers')
import rospy
import serial
import time
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import position
from hardware_interfaces.msg    import compass
from lowlevel_controllers.msg   import depth_pitch_control
from std_msgs.msg               import Float32
from std_msgs.msg               import Bool

#### DEFINE GLOBAL VARIABLES ####
global flag
global controller_onOff

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def set_params():
    global DPC
    global L_th
    global cr_approx
    global Ltf_nose
    
    ### General ###
    DPC.deadzone_Depth = 0   # deadzone of the depth error [metre]
    DPC.deadzone_Pitch = 0      # deadzone of the pitch error [degree]
    
    DPC.Depth_Pgain = 500000.00
    DPC.Depth_Igain = 4000.00
    DPC.Depth_Dgain = -1000000.00
    
    DPC.Pitch_Pgain = 10.00
    DPC.Pitch_Igain = 0.00
    DPC.Pitch_Dgain = 1.00
    
    DPC.Thrust_Smax = 1000       # 1000 # maximum thruster setpoint    

    # determine relative arm lengths for thrust allocation
    L_th = 1.06        # distance between two vertical thrusters [metre]: measured
    cr_approx = 1.15           # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
    Ltf_nose = 0.28    # location of the front vertical thruster from nose: measured
    DPC.crMax = 0.15        # maximum cr diviation [metre]: chosen

################################################################################
########## THRUST CONTROLLER ###################################################
################################################################################

def thrust_controller(error_depth, int_error_depth, der_error_depth, error_pitch, int_error_pitch, der_error_pitch):
    global DPC
    global L_th
    global cr_approx
    global Ltf_nose
    
    if numpy.abs(error_depth) > DPC.deadzone_Depth:
    
        DPC.Depth_Pterm = error_depth*DPC.Depth_Pgain
        DPC.Depth_Iterm = int_error_depth*DPC.Depth_Igain
        DPC.Depth_Dterm = der_error_depth*DPC.Depth_Dgain
        
        DPC.Depth_Thrust = DPC.Depth_Pterm + DPC.Depth_Iterm + DPC.Depth_Dterm # determine a required generalised force to bring the AUV down to a desired depth
        
        if numpy.abs(error_pitch) > DPC.deadzone_Pitch:        
        
            DPC.Pitch_Pterm = error_pitch*DPC.Pitch_Pgain
            DPC.Pitch_Iterm = int_error_pitch*DPC.Pitch_Igain
            DPC.Pitch_Dterm = der_error_pitch*DPC.Pitch_Dgain

            cr = DPC.Pitch_Pterm + DPC.Pitch_Iterm + DPC.Depth_Dterm # determine a deviation of a center of rotation to stabilise the pitching
            cr = limits(cr, -DPC.crMax, DPC.crMax)       #Function to contrain within defined limits

        else:
            cr = 0
        
        # determine relative arm lengths for thrust allocation
        DPC.cr = cr
        cr = cr_approx + numpy.sign(DPC.Depth_Thrust)*cr           # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
        Ltf = cr-Ltf_nose   # Moment arm of front vertical thruster from the cr [metre]
        Ltr = L_th-Ltf      # Moment arm of rear vertical thruster from the cr [metre]
        
        ## distribute a generalised force onto each thruster based on a relative arm length
        thruster0 = float(DPC.Depth_Thrust)/float(Ltf)
        thruster1 = float(DPC.Depth_Thrust)/float(Ltr)

        DPC.thruster0 = int(numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5) # according to a relationship between thrust and rpm
        DPC.thruster1 = int(numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5) # according to a relationship between thrust and rpm
        # if a setpoint of one thruster goes beyond the limit. it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        if numpy.abs(DPC.thruster0) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(DPC.thruster0))
            DPC.thruster0 = int(DPC.thruster0*scale_factor)
            DPC.thruster1 = int(DPC.thruster1*scale_factor)
        if numpy.abs(DPC.thruster1) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(DPC.thruster1))
            DPC.thruster0 = int(DPC.thruster0*scale_factor) 
            DPC.thruster1 = int(DPC.thruster1*scale_factor)
    else:
    
        DPC.thruster0 = 0
        DPC.thruster1 = 0
        
    return [DPC.thruster0, DPC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global flag
        global controller_onOff
        global speed
        global DPC
        global depth_der # depth derivative from PT-type filter in compass_oceanserver.py
        
        flag             = False
        speed            = 0
        controller_onOff = Bool()
        delta_t          = 0.1
        set_params()
                
        time_zero        = time.time()        
        # On first loop, initialize relevant parameters in system_state updaters
        [error_depth, int_error_depth] = system_state_depth(-1,0,0,0)
        [error_pitch, int_error_pitch, der_error_pitch] = system_state_pitch(-1,0,0)
        
        while not rospy.is_shutdown():

            dt = time.time() - time_zero                          # Calculate the elapse time since last calculation
            
            if dt >= delta_t and controller_onOff == True:

                time_zero = time.time()
                
                # get sampling
                depth_current = DPC.depth # depth filtered by PT_filter in compass_oceanserver.py
                der_error_depth = depth_der # derivative depth filtered by PT_filter in compass_oceanserver.py
                print depth_der
                depth_demand = DPC.depth_demand
                pitch_current = DPC.pitch # pitch angle measured by xsens
                pitch_demand = DPC.pitch_demand
                
                # Get system state #
                [error_depth, int_error_depth] = system_state_depth(dt,depth_current,depth_demand,der_error_depth)
                [error_pitch, int_error_pitch, der_error_pitch] = system_state_pitch(dt,pitch_current,pitch_demand)
                
                [thruster0, thruster1] = thrust_controller(error_depth, int_error_depth, der_error_depth, error_pitch, int_error_pitch, der_error_pitch)
                
                # update the heading_control.msg, and this will be subscribed by the logger.py
                pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
                pub_DPC.publish(DPC)
                
##                # verbose activity in thrust_controller
##                str = ">>>>>>>>>>>>>>>>Depth demand is %.2fdeg" %(depth_demand) 
##                rospy.loginfo(str)  
##                str = ">>>>>>>>>>>>>>>>Current depth is %.2fdeg" %(depth_current) 
##                rospy.loginfo(str)
##                str = ">>>>>>>>>>>>>>>>Depth error is %.2fdeg" %(error_depth) 
##                rospy.loginfo(str)
##                str = ">>>>>>>>>>>>>>>>Current pitch is %.2fdeg" %(depth_current) 
##                rospy.loginfo(str)
                str = ">>>>>>>>>>>>>>>>Thruster0 setpoint demand is %d" %(thruster0) 
                rospy.loginfo(str)
                str = ">>>>>>>>>>>>>>>>Thruster1 setpoint demand is %d" %(thruster1) 
                rospy.loginfo(str)
                print ''
                self.__controller.switchDepthOnOff(0)

            else:
                time.sleep(0.01)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def system_state_depth(dt,depth_current,depth_demand,der_error_depth):
    global DPC
    global int_error_depth

### ERROR ###
    error_depth  = depth_demand - depth_current
    
    if dt == -1:
        int_error_depth = 0
               
    else:

### INTEGRAL ###
        int_error_depth += dt*error_depth					# Calculate the integral error       

### DERIVATIVE ###
        # derivative depth is determined using PT-type filter by compass_oceanserver.py.
        
    # update the error terms. These will be subscribed by the logger node.
    DPC.error_depth = error_depth
    DPC.int_error_depth = int_error_depth
    DPC.der_error_depth = der_error_depth

    return [error_depth, int_error_depth]

################################################################################

def system_state_pitch(dt,pitch_current,pitch_demand):
    global DPC
    global int_error_pitch
    global sample_pitch

### ERROR ###
    error_pitch  = pitch_demand - pitch_current

    if dt == -1:
        sample_pitch = numpy.zeros(2)
        int_error_pitch = 0
        der_error_pitch = 0
               
    else:

### INTEGRAL ###
        int_error_pitch += dt*error_pitch					# Calculate the integral error       

### DERIVATIVE ###
        # this simple calculation is good enough for the xsens
        # PID strategy (compute the derivative of pitch error)
        sample_pitch[1] = sample_pitch[0]	                # Shift old values up in the array
        sample_pitch[0] = error_pitch				        # Set first array term to new error value
        der_error_pitch = (sample_pitch[0]-sample_pitch[1])/dt    # Calculate the derivative error
        
    # update the error terms. These will be subscribed by the logger node.
    DPC.error_pitch = error_pitch
    DPC.int_error_pitch = int_error_pitch
    DPC.der_error_pitch = der_error_pitch

    return [error_pitch, int_error_pitch, der_error_pitch]

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

def compass_callback(compass):
    global flag
    global DPC
    global depth_der # depth derivative from PT-type filter in compass_oceanserver.py
    DPC.depth = compass.depth_filt # depth filtered by PT_filter in compass_oceanserver.py
    depth_der = compass.depth_der # derivative depth filtered by PT_filter in compass_oceanserver.py
    DPC.pitch = compass.pitch # pitch angle measured by xsens
    flag = True

def depth_onOff_callback(onOff):
    global flag
    global controller_onOff
    controller_onOff=onOff.data
    flag = True

def depth_demand_callback(depthd):
    global flag
    global DPC
    DPC.depth_demand = depthd.data
    flag = True
    
def pitch_demand_callback(pitchd):
    global flag
    global DPC
    DPC.pitch_demand = pitchd.data
    flag = True

def speed_callback(data):
    global flag
    global speed
    global DPC
    speed = data.forward_vel
    DPC.speed = speed        
    flag = True

################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Heading_controller')
    
    global DPC # Depth-Pitch Control
    DPC = depth_pitch_control()

    rospy.Subscriber('depth_demand', Float32, depth_demand_callback)
    rospy.Subscriber('pitch_demand', Float32, pitch_demand_callback)
    rospy.Subscriber('compass_out', compass, compass_callback)
    rospy.Subscriber('position_dead', position, speed_callback)
    rospy.Subscriber('Depth_onOFF', Bool, depth_onOff_callback)

    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_DPC   = rospy.Publisher('Depth_pitch_controller_values', depth_pitch_control)
    
    rospy.loginfo("Depth-Pitch controller online")

    main_control_loop()
