#=========================
# construct state machine that combine simple states to form a more complicated state
#-------------------------

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

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
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
            
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
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, -1, timeout))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
        
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
            smach.Concurrence.add('GoTurning', GoTurning(self._lib, 0, demandRudder, timeout))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, time_steady, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
        
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
