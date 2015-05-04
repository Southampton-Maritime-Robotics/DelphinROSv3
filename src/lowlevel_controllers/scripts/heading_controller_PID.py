#!/usr/bin/python

"""
A PID-based controller to control the heading for the AUV when moving on a horizontal plane.

######################################
#Modifications
2/2/2015: implement PI-D strategy instead of PID to avoid the spike in derivative term when change the demand. In correspond to this, D_gain has to be negative.
5/4/2015: makesure CS and thruster demands are Integer32

# TODO - make the "CS_controller" less agressive at high forward speeds
# TODO - check if the sway force distribution have been done correctly

"""

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
    global HC
    global Ltf
    global Ltr
    global myUti
    global timeLastDemandMax
    global timeLastCallback
    
    timeLastDemandMax = 1 # [sec] if there is no onOff flag updated within this many seconds, controller will be turnned off
    timeLastCallback = time.time()

    ### General ###
    HC.deadzone   = 1   # deadzone of the heading error [degree]
    
    ### CS Controller ###
    HC.CS_Pgain       = 0.5 # FIXME: tune me kantapon
    HC.CS_Igain       = 0
    HC.CS_Dgain       = -0.3 # D gain has to be negative (c.f. PI-D), FIXME: tune me kantapon
    HC.CS_Smax         = 30
#    HC.CS_min         = -HC.CS_max
    
    ### Thrust Controller ###
    HC.Thrust_Pgain = 10000.00
    HC.Thrust_Igain = 0.00
    HC.Thrust_Dgain = 0.0 # -30000.00 # D gain has to be negative (c.f. PI-D)
    HC.Thrust_Smax  = 800 # 1000 # maximum thruster setpoint, FIXME: unleash me kantapon

    ### Utility Object ###
    myUti = uti()
    
    # determine relative arm lengths for thrust allocation
    L_th = 1.24         # distance between two horizontal thrusters [metre]: measured
    cr = 1.05           # center of rotation on horizontal plane from the AUV nose [metre]: trial-and-error
    Ltf_nose = 0.205    # location of the front horizontal thruster from nose: measured
    Ltf = cr-Ltf_nose   # Moment arm of front horizontal thruster from the cr [metre]
    Ltr = L_th-Ltf      # Moment arm of rear horizontal thruster from the cr [metre]

################################################################################
#### CONTROL SURFACE CONTROLLER ################################################
################################################################################

def CS_controller(error, int_error, der_error):
    global HC
    HC.CS_Pterm      = error*HC.CS_Pgain
    HC.CS_Iterm      = 0 # TODO int_error*HC.CS_Igain
    HC.CS_Dterm      = 0 # TODO der_err*HC.CS_Dgain
# TODO may incorporate a forward speed into a consideration using gain schedualing
# TODO other option: divide the gains by u^2. If the speed is less than a threshold, all gain will be set to zero

    CS_demand = HC.CS_Pterm + HC.CS_Iterm + HC.CS_Dterm
    CS_demand  = myUti.limits(CS_demand,-HC.CS_Smax,HC.CS_Smax)
    
    HC.CS_demand = int(round(CS_demand))

    return HC.CS_demand
    
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
        thruster0 = float(HC.Thrust_heading)/float(Ltf+Ltr) + float(HC.sway_demand)*float(Ltr)/float(Ltf+Ltr)
        thruster1 = -float(HC.Thrust_heading)/float(Ltf+Ltr) + float(HC.sway_demand)*float(Ltf)/float(Ltf+Ltr)
        
        thruster0 = numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5 # according to a relationship between thrust and rpm
        thruster1 = numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5 # according to a relationship between thrust and rpm
        # if a setpoint of one thruster goes beyond the limit. it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        thruster0 = round(thruster0)
        thruster1 = round(thruster1)
        if numpy.abs(thruster0) > HC.Thrust_Smax:
            scale_factor = float(HC.Thrust_Smax)/float(numpy.abs(thruster0))
            thruster0 = thruster0*scale_factor
            thruster1 = thruster1*scale_factor
        if numpy.abs(thruster1) > HC.Thrust_Smax:
            scale_factor = float(HC.Thrust_Smax)/float(numpy.abs(thruster1))
            thruster0 = thruster0*scale_factor 
            thruster1 = thruster1*scale_factor

    else:
        thruster0 = 0.
        thruster1 = 0.
        
    HC.thruster0 = int(round(thruster0))
    HC.thruster1 = int(round(thruster1))
        
    return [HC.thruster0, HC.thruster1]

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():

    #### SETUP ####
        global controller_onOff
        global speed
        global HC

        speed            = 0
        controller_onOff = Bool()
        set_params()

        controlRate = 5. # [Hz]
        r = rospy.Rate(controlRate)
        controlPeriod = 1/controlRate # [sec]
        
        [error, int_error, der_error] = system_state(-1,HC.heading,(HC.heading_demand)%360) # On first loop, initialize relevant parameters
        
        while not rospy.is_shutdown():

            if controller_onOff == True:

                timeRef = time.time()                

                # get sampling
                heading_current = HC.heading
                heading_demand = (HC.heading_demand)%360
                
                # Get system state #
                [error, int_error, der_error] = system_state(controlPeriod,heading_current,heading_demand)

                # Control Surface Controller # Nb CSp = Sternplane port, CSt = Rudder top
                CS_demand = CS_controller(error, int_error, der_error)
                # Thruster controller # 
                [thruster0, thruster1] = thrust_controller(error, int_error, der_error)
                
                # update the heading_control.msg, and this will be subscribed by the logger.py
                pub_tail.publish(cs0 =CS_demand, cs1 = CS_demand)
                pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
                pub_HC.publish(HC)
                
                # verbose activity in thrust_controller
#                str = ">>>>>>>>>>>>>>>>Heading demand is %.2fdeg" %(heading_demand) 
#                rospy.loginfo(str)  
#                str = ">>>>>>>>>>>>>>>>Current heading is %.2fdeg" %(heading_current) 
#                rospy.loginfo(str)
#                str = ">>>>>>>>>>>>>>>>Heading error is %.2fdeg" %(error) 
#                rospy.loginfo(str)
#                str = ">>>>>>>>>>>>>>>>Control surface demand is %d" %(CS_demand) 
#                rospy.loginfo(str)
#                str = ">>>>>>>>>>>>>>>>Thruster0 setpoint demand is %d" %(thruster0) 
#                rospy.loginfo(str)
#                str = ">>>>>>>>>>>>>>>>Thruster1 setpoint demand is %d" %(thruster1) 
#                rospy.loginfo(str)
#                print ''

                if time.time()-timeLastCallback > timeLastDemandMax:
                    controller_onOff = False
                    
                timeElapse = time.time()-timeRef
                
                if timeElapse < controlPeriod:
                    r.sleep()
                else:
                    str = "Heading control rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
                    rospy.logwarn(str)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def system_state(dt,heading_current,heading_demand):
    global HC
    global int_error
    global sample

    if dt == -1:
        sample = numpy.zeros(2)
        error = 0
        int_error = 0
        der_sample = 0
    else:
        ### ERROR ###
        error = myUti.computeHeadingError(heading_demand,heading_current)
        ### INTEGRAL ###
        int_error += dt*error
        ### DERIVATIVE ###
        # PI-D strategy (compute the derivative of only the current heading)
        sample[1] = sample[0]	                # Shift old values up in the array
        sample[0] = heading_current  		    # Set first array term to new error value
        der_sample  = myUti.computeHeadingError(sample[0],sample[1])/dt       # compute error of the sample

    # update the error terms. These will be subscribed by the logger node.
    der_error = der_sample # consider the derivative of sample as the derivative of the error (c.f. PI-D strategy)

    HC.error = error
    HC.int_error = int_error
    HC.der_error = der_error

    return [error, int_error, der_error]

################################################################################
######## UPDATE PARAMETERS FROM TOPICS #########################################
################################################################################

def heading_demand_cb(headingd):
    global HC
    HC.heading_demand = headingd.data
    
def sway_demand_cb(swaydemand):
    global HC
    HC.sway_demand = swaydemand.data

def compass_cb(compass):
    global HC
    HC.heading = compass.heading

def onOff_cb(onOff):
    global controller_onOff
    global timeLastCallback
    controller_onOff=onOff.data
    timeLastCallback = time.time()
    
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
