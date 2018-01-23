#=========================
# construct state machine that combine simple states to form a more complicated state
#-------------------------

from __future__ import division
import rospy
import smach
import smach_ros

## import simple states
from delphin2_mission.basic_states import *

class construct_stateContainer(object):
    def __init__(self, lib, myUti):
        #constructor method - creates objects of type 'HighLevelController'
        self._lib   = lib
        self._myUti = myUti
    
    def track_heading_while_going_forward(self, demandProp, demandHeading, time_steady, timeout):
        '''
        Track heading while moving forward at a constant propeller demand.
        return:
            - succeeded: when stablilising at a desired heading within timeout for time_steady second
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired heading, or when non of above conditions is satisfied
        note1: timeout must be larger than time_steady and also large enough so that the AUV can complete the given task
        '''
        
        def child_term_cb(outcome_map):
            if outcome_map['GoToHeading'] == 'succeeded':
                return True
            else:
                return False
        
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'GoToHeading':'succeeded'},
                                                'aborted':
                                                   {'GoToHeading':'aborted'},
                                                'preempted':
                                                   {'GoToHeading':'preempted'}})
        
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout+2))
            
        return sm_con
        
    def track_depth_while_keeping_heading_and_going_forward(self, demandProp, demandHeading, demandDepth, time_steady, timeout): # TODO: need to verify the code
        '''
        Track a depth demand while maintaining at a desired heading and execute a constant propeller demand.
        return:
            - succeeded: when stablilising at a desired depth within timeout for time_steady second
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired depth, or when non of above conditions is satisfied
        note1: timeout must be larger than time_steady and also large enough so that the AUV can complete the given task
        '''
        
        def child_term_cb(outcome_map):
            if outcome_map['GoToDepth'] == 'succeeded':
                return True
            else:
                return False
        
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'GoToDepth':'succeeded'},
                                                'aborted':
                                                   {'GoToDepth':'aborted'},
                                                'preempted':
                                                   {'GoToDepth':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, -1, timeout+2))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout+2))
        
        return sm_con
        
    def track_altitude_while_keeping_heading_and_going_forward(self, demandProp, demandHeading, demandAltitude, time_steady, timeout): # TODO: need to verify the code
        '''
        Track a altitude while maintaining at a desired heading and execute a constant propeller demand.
        return:
            - succeeded: when time-out is reached
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired altitude, or when none of above conditions satisfied
        timeout must be larger than time steady, and allow time to complete task
        '''
        
        def child_term_cb(outcome_map):
            if outcome_map['GoToAltitude'] == 'succeeded':
                return True
            else:
                return False
        
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'GoToAltitude':'succeeded'},
                                                'aborted':
                                                   {'GoToAltitude':'aborted'},
                                                'preempted':
                                                   {'GoToAltitude':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, -1, timeout+2))
            smach.Concurrence.add('GoToAltitude', GoToAltitude(self._lib, demandAltitude, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout+2))
        
        return sm_con
        
    def helicaly_dive_to_depth(self, demandProp, demandDepth, demandRudder, time_steady, timeout): # TODO: need to verify the code
        '''
        Diving down to a desired depth in with a helical trajectory at a constant rudder demand and propeller demand.
        return:
            - succeeded: when stablilising at a desired depth within timeout for time_steady second
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired depth, or when non of above conditions is satisfied
        note1: timeout must be larger than time_steady and also large enough so that the AUV can complete the given task
        '''
        
        def child_term_cb(outcome_map):
            if outcome_map['GoToDepth'] == 'succeeded':
                return True
            else:
                return False
                
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'GoToDepth':'succeeded'},
                                                'aborted':
                                                   {'GoToDepth':'aborted'},
                                                'preempted':
                                                   {'GoToDepth':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoTurning', GoTurning(self._lib, 0, demandRudder, timeout+2))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout+2))
        
        return sm_con
        
    def turning_at_speed_at_surface(self, demandProp, demandThruster, demandRudder, timeout): # TODO: need to verify the code
        '''
        perform a constant turn at a constant propeller demand for "timeout" seconds
        return:
            - succeeded: when timeout is raised
            - preempted: when backSeatDriver flag is raised
            - aborted: when non of above conditions is satisfied (not being used)
        '''
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   outcome_map={'succeeded':
                                                   {'GoTurning':'succeeded',
                                                    'GoForwards':'succeeded'},
                                                'aborted':
                                                   {'GoForwards':'aborted'},
                                                'preempted':
                                                   {'GoForwards':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoTurning', GoTurning(self._lib, demandThruster, demandRudder, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
        
        return sm_con
        
    def LOS_path_following(self, path, demandProp, timeout):
        '''
        Travel along a path using a line-of-signt approach.
        return:
            - succeeded: when arrived a final waypoint
            - preempted: when backSeatDriver flag is raised
            - aborted: timeout
        '''
        # Create the top level state machine
        sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm.userdata.wp = []
        
        # Open the container, add state and define state transition
        with sm:
            smach.StateMachine.add('REVISE_WAYPOINTS', reviseWaypoints(self._lib, path),
                                                       transitions={'succeeded':'PATH_FOLLOWING', 'aborted':'aborted'},
                                                       remapping={'wp_out':'wp'})
            smach.StateMachine.add('PATH_FOLLOWING', pathFollowingLOS(self._lib, self._myUti, timeout=timeout, demandProp=demandProp),
                                                       transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted'},
                                                       remapping={'wp_in':'wp'})
        return sm
        
    def LOS_path_following_at_depth(self, path, demandProp, demandDepth, timeout): # TODO: need to check this state
        '''
        Travel along a path using a line-of-signt approach at a constant depth.
        return:
            - succeeded: when arrived a final waypoint
            - preempted: when backSeatDriver flag is raised
            - aborted: timeout
        '''
        
        def child_term_cb(outcome_map):
            if outcome_map['PATH_FOLLOWING'] == 'succeeded':
                return True
            else:
                return False
                
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'PATH_FOLLOWING':'succeeded'},
                                                'aborted':
                                                   {'PATH_FOLLOWING':'aborted'},
                                                'preempted':
                                                   {'PATH_FOLLOWING':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('PATH_FOLLOWING', self.LOS_path_following(path, demandProp, timeout))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, -1, timeout))
            smach.Concurrence.add('GoToAltitude', GoToAltitude(self._lib, demandAltitude, -1, timeout))
            
        return sm_con
        
    def manoeuvrint_ZigZag(self, demandProp, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout):
        '''
        Perform a ZigZag manoeuvre for cycleMax times (one left and one right are counted as two cycles).
        return:
            - succeeded: complete operation
            - preempted: when backSeatDriver flag is raised
            - aborted: timeout
        '''
        
        sm_jibbing = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm_jibbing.userdata.cycleCount = 0  # a counter to count how many time jibbing has been done.
        sm_jibbing.userdata.dir = 1         # direction of turning
        # Open the container, add state and define state transition
        with sm_jibbing:
            smach.StateMachine.add('GoJibbing', GoJibbing(self._lib, self._myUti, headingMean, headingAmp, demand_th_hor, demand_cs_ver, cycleMax, timeout/cycleMax), 
                transitions={'succeeded':'succeeded', 'continue':'GoJibbing', 'aborted':'aborted', 'preempted':'preempted'},
                remapping={'cycleCount_out':'cycleCount','dir_out':'dir','cycleCount_in':'cycleCount','dir_in':'dir'})
        
        def child_term_cb(outcome_map):
            if outcome_map['Jibbing'] == 'succeeded':
                return True
            else:
                return False
                
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded':
                                                   {'Jibbing':'succeeded'},
                                                'aborted':
                                                   {'Jibbing':'aborted'},
                                                'preempted':
                                                   {'Jibbing':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('Jibbing', sm_jibbing)
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
            
        return sm_con
