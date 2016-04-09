#=========================
# construct state machine that combine simple states to form a more complicated state
#-------------------------

import rospy
import smach
import smach_ros

## import simple states
from delphin2_mission.basic_states import *

##### spiral down to depth
####    - state required:
####        - GoToDepth
####        - GoTurning
####        - GoForwards
####    - outcome: depend on depth reaching
##### GoToXY at depth
####    - state required:
####        - GoToXY
####        - GoToDepth
####    - outcome: depend on XY reaching, and depth reaching

class construct_stateContainer(object):
    def __init__(self, lib, myUti):
        #constructor method - creates objects of type 'HighLevelController'
        self._lib   = lib
        self._myUti = myUti

    def track_heading_while_going_forward(self,demandProp,demandHeading,timeout):
        '''
        Track heading while moving forward at a constant propeller demand for "timeout" seconds
        return:
            - succeeded: when reaching a desired heading within timeout
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired heading, or when non of above conditions is satisfied
        note1: timeout must be as small as 5sec to constantly verify conditions of the AUV
        note2: when return aborted, the calling state will transit to itself as to repeat this behaviour until retuning succeeded or preempted
        '''
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   outcome_map={'succeeded':
                                                   {'GoToHeading':'succeeded',
                                                    'GoForwards':'succeeded'},
                                                'aborted':
                                                   {'GoToHeading':'aborted'},
                                                'preempted':
                                                   {'GoToHeading':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, -1, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
            
        return sm_con
        
    def track_heading_and_depth_while_going_forward(self, demandProp, demandHeading, demandDepth, timeout): # TODO: need to verify the code
        '''
        Track heading and depth while moving forward at a constant propeller demand for "timeout" seconds
        return:
            - succeeded: when reaching a desired heading and depth within timeout
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired depth, or when non of above conditions is satisfied
        note1: timeout must be as small as 5sec to constantly verify conditions of the AUV
        note2: when return aborted, the calling state will transit to itself as to repeat this behaviour until retuning succeeded or preempted
        '''
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   outcome_map={'succeeded':
                                                   {'GoToHeading':'succeeded',
                                                    'GoToDepth':'succeeded',
                                                    'GoForwards':'succeeded'},
                                                'aborted':
                                                   {'GoToDepth':'aborted'},
                                                'preempted':
                                                   {'GoToDepth':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoToHeading', GoToHeading(self._lib, self._myUti, demandHeading, -1, timeout))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, -1, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
        
        return sm_con
        
    def helical_down_to_depth(self, demandProp, demandDepth, demandRudder, timeout): # TODO: need to verify the code
        '''
        perform a helical manoeuvre at a constant rudder demand and propeller demand while diving down to a desired depth for "timeout" seconds
        return:
            - succeeded: when reaching a desired depth within timeout
            - preempted: when backSeatDriver flag is raised
            - aborted: when timeout before reaching a desired depth, or when non of above conditions is satisfied
        '''
        # Create concurent container with a transition in according to the outcome
        sm_con = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                   default_outcome='aborted',
                                   outcome_map={'succeeded':
                                                   {'GoTurning':'succeeded',
                                                    'GoToDepth':'succeeded',
                                                    'GoForwards':'succeeded'},
                                                'aborted':
                                                   {'GoToDepth':'aborted'},
                                                'preempted':
                                                   {'GoToDepth':'preempted'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('GoTurning', GoToHeading(self._lib, 0, demandRudder, -1, timeout))
            smach.Concurrence.add('GoToDepth', GoToDepth(self._lib, demandDepth, -1, timeout))
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
            smach.Concurrence.add('GoTurning', GoToHeading(self._lib, demandThruster, demandRudder, -1, timeout))
            smach.Concurrence.add('GoForwards', GoForwards(self._lib, demandProp, timeout))
        
        return sm_con
