#!/usr/bin/python

"""
A heading controller for the AUV when moving on a horizontal plane based-on PI-D strategy

# Notes
-control surface may not functioning correctly as the gains has not been tested yet

######################################
#Modifications
2/2/2015: implement PI-D strategy instead of PID to avoid the spike in derivative term when change the demand. In correspond to this, D_gain has to be negative.
5/4/2015: makesure CS and thruster demands are Integer32
21/9/2015: added speed observer

# TODO 
- check if the sway force distribution have been done correctly
- include actuator transition in according to surge speed
- moved speed observer to dead_reckoner

"""

import rospy
import time
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from lowlevel_controllers.msg   import heading_control_PID
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import String
from hardware_interfaces.msg    import status

from delphin2_mission.utilities     import uti

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
    global timeLastDemandProp
    global timeLastDemandProp_lim
    
    timeLastDemandMax = 1 # [sec] if there is no onOff flag updated within this many seconds, controller will be turnned off
    timeLastCallback = time.time()
    timeLastDemandProp = time.time()
    timeLastDemandProp_lim = 1 # [sec] if there is no propeller demand update within this many seconds, the demand will be set to zero

    ### General ###
    HC.deadzone   = 0   # deadzone of the heading error [degree]
    
    ### CS Controller ###
    HC.CS_Pgain       = 8. # FIXME: tune me kantapon
    HC.CS_Igain       = 0
    HC.CS_Dgain       = -13. # D gain has to be negative (c.f. PI-D), FIXME: tune me kantapon
    HC.CS_Smax         = 30
    
    ### Thrust Controller ###
    HC.Thrust_Pgain = rospy.get_param("horizontal/thruster/Pgain")
    HC.Thrust_Igain = rospy.get_param("horizontal/thruster/Igain")
    HC.Thrust_Dgain = rospy.get_param("horizontal/thruster/Dgain") # -30000.00 # D gain has to be negative (c.f. PI-D)
    HC.Thrust_Smax  = rospy.get_param("thruster/SetpointMax") # 1000 # maximum thruster setpoint

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
    HC.CS_Iterm      = int_error*HC.CS_Igain
    HC.CS_Dterm      = der_error*HC.CS_Dgain

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
        HC.Thrust_Pterm = 0.
        HC.Thrust_Dterm = 0.
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
    global propDemand

    propDemand       = 0
    speed            = 0
    controller_onOff = False
    set_params()

    controlRate = 5. # [Hz]
    r = rospy.Rate(controlRate)
    controlPeriod = 1/controlRate # [sec]
    
    [error, int_error, der_error] = system_state(-1,HC.heading,(HC.heading_demand)%360) # On first loop, initialize relevant parameters
    
    # to control a timing for status publishing
    timeZero_status = time.time()
    try:
        dt_status = rospy.get_param('status_timing')
    except:
        dt_status = 2.
    
    while not rospy.is_shutdown():
        # to control a timing for status publishing
        if time.time()-timeZero_status > dt_status:
            timeZero_status = time.time()
            pubStatus.publish(nodeID = 7, status = True)
            
        timeRef = time.time()                
        
        if time.time()-timeLastDemandProp > timeLastDemandProp_lim:
            propDemand = 0
        try:
            speed_current = speedObserver(propDemand, speed, controlPeriod)
            speed = speed_current
            HC.speed = speed
        except:
            speed_current = 0
            speed = speed_current
            HC.speed = speed

        if controller_onOff == True:
            # get sampling
            heading_current = HC.heading
            heading_demand = (HC.heading_demand)%360
            
            # Get system state #
            [error, int_error, der_error] = system_state(controlPeriod,heading_current,heading_demand)

            # Control Surface Controller # Nb CSp = Sternplane port, CSt = Rudder top
            CS_demand = CS_controller(error, int_error, der_error)
            # Thruster controller # 
            [thruster0, thruster1] = thrust_controller(error, int_error, der_error)
            
            # update the heading_control_PID.msg, and this will be subscribed by the logger.py
            pub_tail.publish(cs0 =CS_demand, cs1 = CS_demand)
            pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
            pub_HC.publish(HC)
            
            # watch to inactivate the controller when there is no demand specified
            if time.time()-timeLastCallback > timeLastDemandMax:
                controller_onOff = False
        else:
            HC.CS_demand = 0
            HC.thruster0 = 0
            HC.thruster1 = 0
            pub_HC.publish(HC)
            
        # verify and maintain the control rate
        timeElapse = time.time()-timeRef        
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "Heading control rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse) 
            rospy.logwarn(str)
            pubMissionLog.publish(str)

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
    global controller_onOff
    global timeLastCallback
    HC.heading_demand = headingd.data
    controller_onOff = True
    timeLastCallback = time.time()
    
def sway_demand_cb(swaydemand):
    global HC
    HC.sway_demand = swaydemand.data

def compass_cb(compass):
    global HC
    HC.heading = compass.heading
    
################################################################################
######## SPEED OBSERVER ########################################################
################################################################################

def prop_demand_callback(propd):
    global propDemand
    global timeLastDemandProp
    propDemand = propd.data
    timeLastDemandProp = time.time()

def propeller_model(u_prop,_speed):
    if numpy.abs(u_prop)<10:   # deadband
        F_prop = 0
    else:
        # propeller model based on Turnock2010 e.q. 16.19
        Kt0 = 0.1003
        a = 0.6952
        b = 1.6143
        w_t = 0.36 # wake fraction
        t = 0.11 # thrust deduction
        D = 0.305 # peopeller diameter [m]
        rho = 1000 # water density [kg/m^3]
                
        rps = 0.2748*u_prop - 0.1657 # infer propeller rotation speed from the propeller demand [rps]
                    
        J = _speed *(1-w_t)/rps/D;
        Kt = Kt0*(1 - (J/a)**b );
        F_prop = rho*rps**2*D**4*Kt*(1-t);

    return F_prop

def rigidbodyDynamics(_speed,F_prop):
    m = 79.2 # mass of the AUV [kg]
    X_u_dot = -3.46873716858361 # added mass of the AUV [kg]
    X_u = -16.2208 # linear damping coefficient [kg/s]
    X_uu = -1.2088 # quadratic damping coefficient [kg/m]
    
    acc = (X_u*_speed + X_uu*abs(_speed)*_speed + F_prop)/(m-X_u_dot)
    
    return acc
    
def speedObserver(u_prop,_speed,dt):
    # compute force from a propeller demand
    F_prop = propeller_model(u_prop,_speed)
    # implement Runge-Kutta 4th order to update the AUV speed
    k1 = rigidbodyDynamics(_speed,F_prop)
    k2 = rigidbodyDynamics(_speed+dt/2.*k1,F_prop)
    k3 = rigidbodyDynamics(_speed+dt/2.*k2,F_prop)
    k4 = rigidbodyDynamics(_speed+dt*k3,F_prop)

    speed_change = dt/6.*(k1+2*k2+2*k3+k4)
    _speed = _speed + speed_change
    return _speed
    
################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Heading_controller')
    
    global HC
    HC = heading_control_PID()
   
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)
    rospy.Subscriber('sway_demand', Float32, sway_demand_cb)
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('prop_demand', Int8, prop_demand_callback)
    
    pub_tsl  = rospy.Publisher('TSL_setpoints_horizontal', tsl_setpoints, queue_size=10)
    pub_tail = rospy.Publisher('tail_setpoints_vertical', tail_setpoints, queue_size=10)
    pub_HC   = rospy.Publisher('Heading_controller_values_PID', heading_control_PID, queue_size=10)
    pubMissionLog = rospy.Publisher('MissionStrings', String, queue_size=10)
    pubStatus = rospy.Publisher('status', status, queue_size=10)
    
    rospy.loginfo("Heading controller online")

    main_control_loop()
