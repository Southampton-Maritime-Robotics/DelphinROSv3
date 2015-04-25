#!/usr/bin/python

"""
This node control pitch and depth based on the PI-D strategy.
Pitch bias is used to indirectly control the depth of the AUV via control surfaces when undergoing a surge motion.

######################################
#Modifications
# 2/2/2015: implement PI-D strategy instead of PID to avoid the spike in derivative term when change the demand. In correspond to this, D_gain has to be negative.
# 5/4/2015: force CS and thruster demands to become Integer32

"""

import rospy
import serial
import time
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import position
from hardware_interfaces.msg    import compass
from lowlevel_controllers.msg   import depth_pitch_control
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import Bool

#### from kantapon's folder
import sys
import os.path
basepath = os.path.dirname(__file__)
filepath = os.path.abspath(os.path.join(basepath, '..', '..', 'delphin2_mission/scripts/kantapon'))
sys.path.append(filepath)
from utilities                      import uti

################################################################################
#### CONTROLLER PARAMETERS #####################################################
################################################################################

def set_params():
    global DPC
    global L_th
    global cr_approx
    global Ltf_nose
    global myUti
    global timeLastDemandMax
    global timeLastCallback
    global propDemand
    global crMax
    global crMin
    
    ### General ###
    propDemand = 0
    timeLastDemandMax = 1 # [sec] if there is no onOff flag updated within this many seconds, controller will be turnned off
    timeLastCallback = time.time()

    DPC.deadzone_Depth = 0   # deadzone of the depth error [metre]
    DPC.deadzone_Pitch = 0      # deadzone of the pitch error [degree]
    
    # control surfaces
    DPC.CS_Pgain = 0. # P gain for control surface
    DPC.CS_Igain = 0. # I gain for control surface
    DPC.CS_Dgain = 0. # D gain for control surface
    
    DPC.CS_Smax = 30 # [degree] maximum hydroplane angle
    
    # thruster
    DPC.Depth_Pgain = 700000.00 # FIXME: tune me kantapon
    DPC.Depth_Igain = 50000.00 # FIXME: tune me kantapon
    DPC.Depth_Dgain = -1000000.00 # D gain has to be negative (c.f. PI-D), FIXME: tune me kantapon
    
    DPC.Pitch_Pgain = 0.02 # FIXME: tune me kantapon
    DPC.Pitch_Igain = 0.01 # FIXME: tune me kantapon
    DPC.Pitch_Dgain = -0.01 # D gain has to be negative (c.f. PI-D),FIXME: tune me kantapon
    
    DPC.Thrust_Smax = 1800       # maximum thruster setpoint # FIXME: unleash me kantapon

    DPC.pitchBiasMax = 5. # bias in pitch angle, use to indirectly control depth vis control surfaces [degree]
    DPC.pitchBiasGain = -5. # p gain to compute bias: has to be -VE

    ### determine relative arm lengths for thrust allocation ###
    L_th = 1.06             # distance between two vertical thrusters [metre]: measured
    cr_approx = 0.98        # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
    Ltf_nose = 0.28         # location of the front vertical thruster from nose: measured
#    DPC.crTol = 0.05        # FIXME change the message structure and logger. to bound the cr between front and rear thruster with this gap from the bound [metre]: chosen
    
    crTol = 0.15
    crMax = crTol
    crMin = -crTol
#    crMin = cr_approx - Ltf_nose -L_th + crTol      # length from cr_approx to front thruster
#    crMax = cr_approx - Ltf_nose - crTol            # length from cr_approx to rear thruster
    ### Utility Object ###
    myUti = uti()
    
#################################################################################
##### CONTROL SURFACE CONTROLLER ################################################
#################################################################################

def CS_controller(error_pitch, int_error_pitch, der_error_pitch):
    global DPC
    DPC.CS_Pterm      = error_pitch*DPC.CS_Pgain
    DPC.CS_Iterm      = 0 # TODO int_error*DPC.CS_Igain
    DPC.CS_Dterm      = 0 # TODO der_err*DPC.CS_Dgain

    CS_demand = DPC.CS_Pterm + DPC.CS_Iterm + DPC.CS_Dterm
    CS_demand = myUti.limits(CS_demand,-DPC.CS_Smax,DPC.CS_Smax)
    
    DPC.CS_demand = int(round(CS_demand))

    return DPC.CS_demand
    
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
        
#        DPC.Depth_Thrust = 4*10**2 # TODO remove me
        
        if numpy.abs(error_pitch) > DPC.deadzone_Pitch:        
        
            DPC.Pitch_Pterm = error_pitch*DPC.Pitch_Pgain
            DPC.Pitch_Iterm = int_error_pitch*DPC.Pitch_Igain
            DPC.Pitch_Dterm = der_error_pitch*DPC.Pitch_Dgain

            cr = DPC.Pitch_Pterm + DPC.Pitch_Iterm + DPC.Pitch_Dterm # determine a deviation of a center of rotation to stabilise the pitching

        else:
            cr = 0
        
        cr = myUti.limits(-cr, crMin, crMax)       # Function to contrain within defined limits (w.r.t. cr_approx)
        
#        cr = 0 # FIXME remove this line
        
        DPC.cr = cr
        cr = cr_approx - cr # TODO check if it can handle the change in depth demand. numpy.sign(DPC.Depth_Thrust)*cr           # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
        Ltf = cr-Ltf_nose   # Moment arm of front vertical thruster from the cr [metre]
        Ltr = L_th-Ltf      # Moment arm of rear vertical thruster from the cr [metre]
        print [crMax-crMin, cr, Ltf, Ltr]
        
        # TODO remove this block
        DPC.Ltf = Ltf
        DPC.Ltr = Ltr
        DPC.crNose = cr
        
        ## distribute a generalised force onto each thruster based on a relative arm length
        thruster0 = float(DPC.Depth_Thrust)/float(Ltf)
        thruster1 = float(DPC.Depth_Thrust)/float(Ltr)

        thruster0 = numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5 # according to a relationship between thrust and rpm
        thruster1 = numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5 # according to a relationship between thrust and rpm
        # if a setpoint of one thruster goes beyond the limit. it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        thruster0 = round(thruster0)
        thruster1 = round(thruster1)
        if numpy.abs(thruster0) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(thruster0))
            thruster0 = thruster0*scale_factor
            thruster1 = thruster1*scale_factor
        if numpy.abs(thruster1) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(thruster1))
            thruster0 = thruster0*scale_factor 
            thruster1 = thruster1*scale_factor
    else:
    
        DPC.thruster0 = 0.
        DPC.thruster1 = 0.
        
    DPC.thruster0 = int(round(thruster0))
    DPC.thruster1 = int(round(thruster1))
    
    return [DPC.thruster0, DPC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global controller_onOff
        global DPC
        global depth_der # depth derivative from PT-type filter in compass_oceanserver.py
        
        controller_onOff = Bool()
        set_params()
        
        controlRate = 10. # [Hz]
        r = rospy.Rate(controlRate)
        controlPeriod = 1/controlRate # [sec]
        
        # On first loop, initialize relevant parameters in system_state updaters
        [error_depth, int_error_depth] = system_state_depth(-1,0,0,0)
        [error_pitch, int_error_pitch, der_error_pitch] = system_state_pitch(-1,0,0,0)
        
        while not rospy.is_shutdown():
            
            if controller_onOff == True:

                timeRef = time.time()
                
                # get sampling
                depth_current = DPC.depth # depth filtered by PT_filter in compass_oceanserver.py
                der_error_depth = depth_der # derivative depth filtered by PT_filter in compass_oceanserver.py
                depth_demand = DPC.depth_demand
                pitch_current = DPC.pitch # pitch angle measured by xsens
                pitch_demand = DPC.pitch_demand
                
                # Get system state #
                DPC.pitchBias = determinePitchBias(depth_current,depth_demand)
                [error_depth, int_error_depth] = system_state_depth(controlPeriod,depth_current,depth_demand,der_error_depth)
                [error_pitch, int_error_pitch, der_error_pitch] = system_state_pitch(controlPeriod,pitch_current,pitch_demand,DPC.pitchBias)
                
                CS_demand = CS_controller(error_pitch, int_error_pitch, der_error_pitch)
                [thruster0, thruster1] = thrust_controller(error_depth, int_error_depth, der_error_depth, error_pitch, int_error_pitch, der_error_pitch)
                
                # update the heading_control.msg, and this will be subscribed by the logger.py
#                pub_tail.publish(cs0 =CS_demand, cs1 = CS_demand)
#                pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
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
                
                if time.time()-timeLastCallback > timeLastDemandMax:
                    controller_onOff = False
    
                timeElapse = time.time()-timeRef
                if timeElapse < controlPeriod:
                    r.sleep()
                else:
                    str = "Pitch-Depth control rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse)
                    rospy.logwarn(str)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def determinePitchBias(depth_current,depth_demand):
    if propDemand > 10:
        pitchBias = DPC.pitchBiasGain*(depth_demand-depth_current)
        myUti.limits(pitchBias,-DPC.pitchBiasMax,DPC.pitchBiasMax)
    else:
        pitchBias = 0
    return pitchBias

def system_state_depth(dt,depth_current,depth_demand,der_error_depth):
    global DPC
    global int_error_depth

    if dt == -1:
        error_depth = 0
        int_error_depth = 0
        der_error_depth = 0
    else:
        ### ERROR ###
        error_depth  = depth_demand - depth_current
        ### INTEGRAL ###
        int_error_depth += dt*error_depth					# Calculate the integral error       
        ### DERIVATIVE ###
        # derivative depth is determined using PT-type filter by compass_oceanserver.py.
        # remember, this is the PI-D strategy that use derivative of actual depth rathen than the derivative of err_depth

    # update the error terms. These will be subscribed by the logger node.
    DPC.error_depth = error_depth
    DPC.int_error_depth = int_error_depth
    DPC.der_error_depth = der_error_depth

    return [error_depth, int_error_depth]

################################################################################

def system_state_pitch(dt,pitch_current,pitch_demand,pitchBias):
    global DPC
    global int_error_pitch
    global sample_pitch

    if dt == -1:
        sample_pitch = numpy.zeros(2)
        error_pitch = 0
        der_error_pitch = 0
        int_error_pitch = 0               
    else:
        ### ERROR ###
        error_pitch  = pitch_demand - pitch_current + pitchBias
        ### INTEGRAL ###
        int_error_pitch += dt*error_pitch
        ### DERIVATIVE ###
        # this simple calculation is good enough for the xsens
        # PI-D strategy (compute the derivative of pitch)
        sample_pitch[1] = sample_pitch[0]	                # Shift old values up in the array
        sample_pitch[0] = error_pitch				        # Set first array term to new error value
        der_error_pitch = (sample_pitch[0]-sample_pitch[1])/dt    # Calculate the derivative error
        
    # update the error terms. These will be subscribed by the logger node.
    DPC.error_pitch = error_pitch
    DPC.int_error_pitch = int_error_pitch
    DPC.der_error_pitch = der_error_pitch

    return [error_pitch, int_error_pitch, der_error_pitch]

################################################################################
######## UPDATE PARAMETERS FROM TOPICS #########################################
################################################################################

def compass_callback(compass):
    global DPC
    global depth_der # depth derivative from PT-type filter in compass_oceanserver.py
    DPC.depth = compass.depth_filt # depth filtered by PT_filter in compass_oceanserver.py
    depth_der = compass.depth_der # derivative depth filtered by PT_filter in compass_oceanserver.py
    DPC.pitch = compass.pitch # pitch angle measured by xsens

def depth_onOff_callback(onOff):
    global controller_onOff
    global timeLastCallback
    controller_onOff=onOff.data
    timeLastCallback = time.time()

def depth_demand_callback(depthd):
    global DPC
    DPC.depth_demand = depthd.data
    
def pitch_demand_callback(pitchd):
    global DPC
    DPC.pitch_demand = pitchd.data
    
def prop_demand_callback(propd):
    global propDemand
    propDemand = propd.data

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
    rospy.Subscriber('Depth_onOFF', Bool, depth_onOff_callback)
    rospy.Subscriber('prop_demand', Int8, prop_demand_callback)
    
    pub_tail = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_DPC   = rospy.Publisher('Depth_pitch_controller_values', depth_pitch_control)
    
    rospy.loginfo("Depth-Pitch controller online")

    main_control_loop()
