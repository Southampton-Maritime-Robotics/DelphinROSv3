#!/usr/bin/python

"""
A depth-pitch controller based on the PI-D strategy.

Note:
- control surface and pitchBias are not in use yet. This will be very important part when the AUV operates at forward speeds.
- pitch_Dgain makes the system becomes worse possibly due to the delay response of the thruster control boards

######################################
#Modifications
2/2/2015: implement PI-D strategy instead of PID to avoid the spike in derivative term when change the demand. In correspond to this, D_gain has to be negative.
5/4/2015: force CS and thruster demands to become Integer32
8/7/2015: removed pitch_demand_callback: Maked it zero always
18/8/2015: added anti-windup scheme to when computing the depth integral term
21/9/2015: added speed observer

# TODO
- moved speed observer to dead_reckoner

"""

import rospy
import serial
import time
import math
import numpy
from hardware_interfaces.msg    import tsl_setpoints
from hardware_interfaces.msg    import tail_setpoints
from hardware_interfaces.msg    import compass
from hardware_interfaces.msg    import depth
from lowlevel_controllers.msg   import depth_pitch_control_PID
from std_msgs.msg               import Float32
from std_msgs.msg               import Int8
from std_msgs.msg               import Bool
from std_msgs.msg               import String
from hardware_interfaces.msg import status

from delphin2_mission.utilities import uti
from lowlevel_controllers.depth_pitch_controller_PID import DepthPitchPID

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
    global windup_err_thr
    global windup_der_err_thr
    global int_error_depth
    global int_error_depth_lim
    global int_error_pitch_th
    global int_error_pitch_cs
    global int_error_pitchBias
    global speed
    global timeLastDemandProp
    global timeLastDemandProp_lim
    
    ### General ###
    propDemand = 0
    speed = 0
    timeLastDemandMax = 1 # [sec] if there is no onOff flag updated within this many seconds, controller will be turnned off
    timeLastCallback = time.time()
    timeLastDemandProp = time.time()
    timeLastDemandProp_lim = 1 # [sec] if there is no propeller demand update within this many seconds, the demand will be set to zero

    DPC.deadzone_Depth = 0   # deadzone of the depth error [metre]
    DPC.deadzone_Pitch = 0      # deadzone of the pitch error [degree]
    
    # control surfaces
    DPC.CS_Pgain = 5 # P gain for control surface
    DPC.CS_Igain = 1.5 # 0.5 # I gain for control surface
    DPC.CS_Dgain = -10 # D gain for control surface
    
    DPC.CS_Smax = 30 # [degree] maximum hydroplane angle
    
    # thruster
    DPC.Depth_Pgain = rospy.get_param("vertical/thruster/depth/Pgain")
    DPC.Depth_Igain = rospy.get_param("vertical/thruster/depth/Igain")
    DPC.Depth_Dgain = rospy.get_param("vertical/thruster/depth/Dgain")
    
    DPC.Pitch_Pgain = rospy.get_param("vertical/thruster/pitch/Pgain")
    DPC.Pitch_Igain = rospy.get_param("vertical/thruster/pitch/Igain")
    DPC.Pitch_Dgain = rospy.get_param("vertical/thruster/pitch/Dgain") # D gain has to be negative (c.f. PI-D)
    
    DPC.Thrust_Smax = rospy.get_param("/thruster/SetpointMax") # maximum thruster setpoint

    DPC.pitchBiasMax = 10. # bias in pitch angle, use to indirectly control depth vis control surfaces [degree]
    DPC.pitchBiasGain_P = 8. # p gain to compute pitch bias
    DPC.pitchBiasGain_I = 2. # i gain to compute pitch bias
    DPC.pitchBiasGain_D = -10. # d gain to compute pitch bias
    if DPC.pitchBiasGain_I != 0:
        int_error_pitchBias_max = 6./DPC.pitchBiasGain_I
    ### determine relative arm lengths for thrust allocation ###
    L_th = 1.06             # distance between two vertical thrusters [metre]: measured
    cr_approx = 0.85 # 1.05 # 0.98        # center of rotation on vertical plane from the AUV nose [metre]: trial-and-error
    Ltf_nose = 0.28         # location of the front vertical thruster from nose: measured
    crTol = 0.2 # to bound the cr between front and rear thruster with this gap from the bound [metre]: chosen
    crMax = crTol
    crMin = -crTol
    
    # initialise integrators
    int_error_depth = 1.2e6/DPC.Depth_Igain # initialise the integrator for depth error signal
    int_error_depth_lim = 2.5e6/DPC.Depth_Igain # chosen
    int_error_pitch_th = 0. # initialise the integrator for pitch error signal used in thruster control allocation
    int_error_pitch_cs = 0./DPC.CS_Igain # initialise the integrator for pitch error signal used in control surface control law
    int_error_pitchBias = 2./DPC.pitchBiasGain_I # initialise the integrator for pitchBias
    

    ### Parameter for Anti-Windup Scheme ###
    windup_err_thr = 0.8 # if abs(depth_error) goes beyond this and,
    windup_der_err_thr = 0.05 # if abs(depth_error) goes beyond this, the the integrator will be paused

    ### Utility Object ###
    myUti = uti()
    
#################################################################################
##### CONTROL SURFACE CONTROLLER ################################################
#################################################################################

def CS_controller(error_pitch, der_error_pitch, dt, w_cs):
    global DPC
    global int_error_pitch_cs
    
    # update integrator and apply saturation, reset the integrator if propDemand = 0
    if propDemand != 0:
        int_error_pitch_cs_max = DPC.CS_Smax/DPC.CS_Igain*8./30.
        int_error_pitch_cs += dt*error_pitch
        if DPC.CS_Igain != 0: # do the update only when the integral gain is specified
            int_error_pitch_cs = myUti.limits(int_error_pitch_cs, -int_error_pitch_cs_max, int_error_pitch_cs_max)
    else:
        int_error_pitch_cs = 0./DPC.CS_Igain
    
    DPC.CS_Pterm      = error_pitch*DPC.CS_Pgain
    DPC.CS_Iterm      = int_error_pitch_cs*DPC.CS_Igain
    DPC.CS_Dterm      = der_error_pitch*DPC.CS_Dgain

    CS_demand = DPC.CS_Pterm + DPC.CS_Iterm + DPC.CS_Dterm
    CS_demand = w_cs * CS_demand # apply a weight for speed transition
    CS_demand = myUti.limits(CS_demand,-DPC.CS_Smax,DPC.CS_Smax)
    
    DPC.CS_demand = int(round(CS_demand))

    return DPC.CS_demand
    
################################################################################
########## THRUST CONTROLLER ###################################################
################################################################################

def thrust_controller(depth_current, error_depth, der_error_depth, error_pitch, der_error_pitch, dt, w_th):
    global DPC
    global L_th
    global cr_approx
    global Ltf_nose
    global flag_pitch_clamping
    global int_error_depth
    global int_error_pitch_th
    
    if numpy.abs(error_depth) > DPC.deadzone_Depth:    
        # conditional integrator for depth signal
        flag_depth_int_th = 0
        if numpy.abs(error_depth)<windup_err_thr and numpy.abs(der_error_depth)<windup_der_err_thr:
            int_error_depth += dt*error_depth # Calculate the integral error
            flag_depth_int_th = 1
        elif depth_current < 0.4 and numpy.abs(der_error_depth)<windup_der_err_thr: # guarantee the integrators is active when the sub is near the water surface
            int_error_depth += dt*error_depth
            flag_depth_int_th = 1
####        int_error_depth += dt*error_depth # Calculate the integral error
        
        DPC.Depth_Pterm = error_depth*DPC.Depth_Pgain
        DPC.Depth_Iterm = int_error_depth*DPC.Depth_Igain
        DPC.Depth_Dterm = der_error_depth*DPC.Depth_Dgain
        
        DPC.Depth_Thrust = DPC.Depth_Pterm + DPC.Depth_Iterm + DPC.Depth_Dterm # determine a required generalised force to bring the AUV down to a desired depth
        DPC.Depth_Thrust = w_th * DPC.Depth_Thrust # apply a weight for speed transition
        
        if numpy.abs(error_pitch) > DPC.deadzone_Pitch:

            # update the integrator and apply saturation
            if propDemand == 0: # update the integrator only when the thruster is not spining
                int_error_pitch_th += dt*error_pitch
                int_error_pitch_th = myUti.limits(int_error_pitch_th, crMin/DPC.Pitch_Igain/2., crMax/DPC.Pitch_Igain/2.)
        
            DPC.Pitch_Pterm = error_pitch*DPC.Pitch_Pgain
            DPC.Pitch_Iterm = int_error_pitch_th*DPC.Pitch_Igain
            DPC.Pitch_Dterm = der_error_pitch*DPC.Pitch_Dgain

            cr = DPC.Pitch_Pterm + DPC.Pitch_Iterm + DPC.Pitch_Dterm # determine a deviation of a center of rotation to stabilise the pitching

        else:
            cr = 0
            
        cr_ref = cr
        cr = myUti.limits(-cr, crMin, crMax)       # Function to contrain within defined limits (w.r.t. cr_approx)
        
        DPC.cr = cr
        cr = cr_approx - cr #
        Ltf = cr-Ltf_nose   # Moment arm of front vertical thruster from the cr [metre]
        Ltr = L_th-Ltf      # Moment arm of rear vertical thruster from the cr [metre]
        
        ## distribute a generalised force onto each thruster based on a relative arm length
        thruster0 = float(DPC.Depth_Thrust)*float(Ltr)/float(Ltf+Ltr)
        thruster1 = float(DPC.Depth_Thrust)*float(Ltf)/float(Ltf+Ltr)

        thruster0 = numpy.sign(thruster0)*(numpy.abs(thruster0))**0.5 # according to a relationship between thrust and rpm
        thruster1 = numpy.sign(thruster1)*(numpy.abs(thruster1))**0.5 # according to a relationship between thrust and rpm

        # If a setpoint of one thruster goes beyond the limit, 
        # it will be saturated and the other one will be scaled down proportionally in order to scale down torque.
        thruster0 = round(thruster0)
        thruster1 = round(thruster1)
        flag_sat = 0

        if numpy.abs(thruster0) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(thruster0))
            thruster0 = thruster0*scale_factor
            thruster1 = thruster1*scale_factor
            flag_sat = 1
        if numpy.abs(thruster1) > DPC.Thrust_Smax:
            scale_factor = float(DPC.Thrust_Smax)/float(numpy.abs(thruster1))
            thruster0 = thruster0*scale_factor 
            thruster1 = thruster1*scale_factor
            flag_sat = 1
        
        # if the thruster is saturated or the propeller is spining, undo integrator update as to prevent integral windup
        if flag_depth_int_th == 1:
            if flag_sat:
                int_error_depth -= dt*error_depth
        # bound the integrator only in one direction
        if int_error_depth > int_error_depth_lim: 
            int_error_depth = int_error_depth_lim    
    
    else:
        thruster0 = 0
        thruster1 = 0

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
    global depth_der # depth derivative from PT-type filter in depth_transducer.py
    global speed
    global propDemand
    
    controller_onOff = False
    set_params()
    
    controlRate = 5. # [Hz]
    r = rospy.Rate(controlRate)
    controlPeriod = 1/controlRate # [sec]
    
    # On first loop, initialize relevant parameters in system_state updaters
    error_depth = system_state_depth(-1,0,0,0)
    [error_pitch, der_error_pitch] = system_state_pitch(-1,0,0,0)
    
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
            pubStatus.publish(nodeID = 8, status = True)

        timeRef = time.time()
        # regulary update the AUV speed in according to the propeller demand

        if time.time()-timeLastDemandProp > timeLastDemandProp_lim:
            propDemand = 0
        try:
            speed_current = speedObserver(propDemand, speed, controlPeriod)
            speed = speed_current
            DPC.speed = speed
        except:
            speed_current = 0
            speed = speed_current
            DPC.speed = speed
        
        if controller_onOff == True:
            # get sampling
            depth_current = DPC.depth # depth filtered by PT_filter in depth_transducer.py
            der_error_depth = depth_der # derivative depth filtered by PT_filter in depth_transducer.py
            depth_demand = DPC.depth_demand
            pitch_current = DPC.pitch # pitch angle measured by xsens
            pitch_demand = DPC.pitch_demand

            # get system state
            error_depth = system_state_depth(controlPeriod,depth_current,depth_demand,der_error_depth)
            DPC.pitchBias = determinePitchBias(error_depth,der_error_depth,controlPeriod)
            [error_pitch, der_error_pitch] = system_state_pitch(controlPeriod,pitch_current,pitch_demand,DPC.pitchBias)
            [w_th,w_cs] = state.determineActuatorWeight(speed_current,depth_current)
            
            # determine actuator demands
            CS_demand = CS_controller(error_pitch, 
                                      der_error_pitch, 
                                      controlPeriod, 
                                      w_cs)
            [thruster0, thruster1] = thrust_controller(depth_current, 
                                                       error_depth, 
                                                       der_error_depth, 
                                                       error_pitch, 
                                                       der_error_pitch, 
                                                       controlPeriod, 
                                                       w_th)
            
            # update the depth_pitch_control_PID.msg, and this will be subscribed by the logger.py
            pub_tail.publish(cs0 =CS_demand, cs1 = CS_demand)
            pub_tsl.publish(thruster0 = thruster0, thruster1 = thruster1)
            pub_DPC.publish(DPC)
            
            # watchdog to deactivate the controller when there is no demand specified
            if time.time()-timeLastCallback > timeLastDemandMax:
                controller_onOff = False
        else:
            DPC.CS_demand = 0
            DPC.thruster0 = 0
            DPC.thruster1 = 0
            pub_DPC.publish(DPC)

        # verify and maintain the control rate
        timeElapse = time.time()-timeRef
        if timeElapse < controlPeriod:
            r.sleep()
        else:
            str = "Pitch-Depth control rate does not meet the desired value of %.2fHz: actual control rate is %.2fHz" %(controlRate,1/timeElapse)
            rospy.logwarn(str)
            pubMissionLog.publish(str)

################################################################################
######## CALCULATE CURRENT SYSTEM STATES #######################################
################################################################################

def determineActuatorWeight(_speed,_depth):
    # DDD
    # has been moved to src
    """
    Calculate Thruster and control surface weight based on forwards speed.
    both values lie between 0 and 1
    thruster weight decreases with speed, control surface weight increases with speed
    """

    U_star_th = 0.9;
    w_delta_th = 0.03;
    w_th = 1-0.5*(math.tanh((_speed-U_star_th)/w_delta_th)+1);

    U_star_cs = 0.5;
    w_delta_cs = 0.04;
    w_cs = 0.5*(math.tanh((_speed-U_star_cs)/w_delta_cs)+1);
    
    return [w_th, w_cs]

def determinePitchBias(error_depth,der_error_depth,dt):
    global int_error_pitchBias
    
    if propDemand != 0:
        # update integrator for pitchBias and apply saturation
        int_error_pitchBias += dt*error_depth
        if DPC.pitchBiasGain_I != 0:
            int_error_pitchBias_max = 6./DPC.pitchBiasGain_I
            int_error_pitchBias = myUti.limits(int_error_pitchBias,0,int_error_pitchBias_max)
        
        pitchBias = DPC.pitchBiasGain_P*error_depth + DPC.pitchBiasGain_I*int_error_pitchBias + DPC.pitchBiasGain_D*der_error_depth
        pitchBias_ref = pitchBias
        pitchBias = myUti.limits(pitchBias,-DPC.pitchBiasMax,DPC.pitchBiasMax)
    else:
        int_error_pitchBias = 2./DPC.pitchBiasGain_I
        pitchBias = 0
    
    DPC.pitchBias_Iterm = int_error_pitchBias
            
    return pitchBias

def system_state_depth(dt,depth_current,depth_demand,der_error_depth):
    global DPC

    if dt == -1:
        error_depth = 0
        der_error_depth = 0
    else:
        ### ERROR ###
        error_depth  = depth_demand - depth_current
        ### INTEGRAL ###
        # integrater is moved into thrust_control()
        ### DERIVATIVE ###
        # derivative depth is determined using PT-type filter by depth_transducer.py.
        # remember, this is the PI-D strategy that use derivative of actual depth rathen than the derivative of err_depth

    # update the error terms. These will be subscribed by the logger node.
    DPC.error_depth = error_depth
    DPC.der_error_depth = der_error_depth

    return error_depth

################################################################################

def system_state_pitch(dt,pitch_current,pitch_demand,pitchBias):
    global DPC
    global sample_pitch

    if dt == -1:
        sample_pitch = numpy.zeros(2)
        error_pitch = 0
        der_error_pitch = 0
    else:
        ### ERROR ###
        error_pitch  = pitch_demand - pitch_current - pitchBias
        ### INTEGRAL ###
        # integrator is moved to thrust_controller as int_error_pitch_th and CS_controller as int_error_pitch_cs
        ### DERIVATIVE ###
        # this simple calculation is good enough for the xsens
        # PI-D strategy (compute the derivative of pitch)
        sample_pitch[1] = sample_pitch[0]	                # Shift old values up in the array
        sample_pitch[0] = pitch_current				        # Set first array term to new error value
        der_error_pitch = (sample_pitch[0]-sample_pitch[1])/dt    # Calculate the derivative error
        
    # update the error terms. These will be subscribed by the logger node.
    DPC.error_pitch = error_pitch
    DPC.der_error_pitch = der_error_pitch

    return [error_pitch, der_error_pitch]

################################################################################
######## UPDATE PARAMETERS FROM TOPICS #########################################
################################################################################

def compass_callback(compass):
    global DPC
    DPC.pitch = compass.pitch # pitch angle measured by xsens
    
def depth_callback(data):
    global DPC
    global depth_der # depth derivative from PT-type filter in depth_transducer.py
    DPC.depth = data.depth_filt # depth filtered by PT_filter in depth_transducer.py
    depth_der = data.depth_der # derivative depth filtered by PT_filter in depth_transducer.py

def depth_demand_callback(depthd):
    global DPC
    global controller_onOff
    global timeLastCallback
    DPC.depth_demand = depthd.data
    controller_onOff = True
    timeLastCallback = time.time()
    
####def pitch_demand_callback(pitchd):
####    global DPC
####    DPC.pitch_demand = pitchd.data
        
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
    
    global DPC # Depth-Pitch Control    # QUESTION SOPHIA: Why is this global? It is an object so it should be available throughout
    DPC = depth_pitch_control_PID()
    state = DepthPitchPID()
    
    rospy.Subscriber('depth_demand', Float32, depth_demand_callback)
    DPC.pitch_demand = 0 # rospy.Subscriber('pitch_demand', Float32, pitch_demand_callback)
    rospy.Subscriber('compass_out', compass, compass_callback)
    rospy.Subscriber('depth_out', depth, depth_callback)
    rospy.Subscriber('prop_demand', Int8, prop_demand_callback)
    
    pub_tail = rospy.Publisher('tail_setpoints_horizontal', tail_setpoints)
    pub_tsl  = rospy.Publisher('TSL_setpoints_vertical', tsl_setpoints)
    pub_DPC  = rospy.Publisher('Depth_pitch_controller_values_PID', depth_pitch_control_PID)
    pubMissionLog = rospy.Publisher('MissionStrings', String)
    pubStatus = rospy.Publisher('status', status)
    
    rospy.loginfo("Depth-Pitch controller online")

    main_control_loop()
