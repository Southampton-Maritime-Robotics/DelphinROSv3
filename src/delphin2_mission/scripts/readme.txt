guideline for using state machine

taskDepthHeadingTracking
- use concurrent state machine with GoToDepth and GoToHeading
- taskDepthHeadingTracking return:
    - preeampted: if either GoToDepth or GoToHeading return preempted - terminate mission
    - aborted: if either GoToDepth or GoToHeading return aborted - start over the taskDepthHeadingTraking
    - succeeded: if both GoToDepth and GoToHeading return succeeded - move onto the next task
- GoToDepth:
    - time steady: -1
    - timeout: 5, for example, to onstantly verify the outcome every 5 sec
- GoToHeading:
    - time steady: -1
    - timeout: 5, for example, to onstantly verify the outcome every 5 sec
    
taskDivingHelix
- use concurrent state machine with GoToDepth, GoTurning and GoForwards
- taskDivingHelix return:
    - preeampted: if either GoToDepth, GoTurning or GoForwards return preempted - terminate mission
    - aborted: if either GoToDepth return aborted - start over the taskDivingHelix
    - succeeded: if both GoToDepth return succeeded - move onto the next task
- GoToDepth:
    - time steady: -1
    - timeout: 5, for example, to onstantly verify the outcome every 5 sec
- GoTurning:
    - define a suitable rudder angle and set horizontal thruster demand to zero
    - timeout: 5, for example, to onstantly verify the outcome every 5 sec
- GoForwards:
    - use a suitable propeller demand
    - timeout: 5, for example, to onstantly verify the outcome every 5 sec
    
taskGoForwardsAtDepth: at a certain heading.
- use concurrent state machine with GoToDepth, GoToHeading and GoForwards
- taskGoForwardsAtDepth return:
    - preeampted: if either GoToDepth, GoToHeading or GoForwards return preempted - terminate mission
    - aborted: not in use but set to return succeeded - move onto the next task
    - succeeded: if both GoToDepth return succeeded - move onto the next task
- GoToDepth:
    - time steady: -1
    - timeout: as long as it has to be
- GoToHeading:
    - set heading demand to None as to track the current heading; otherwise, specify the heading demand
    - time steady: -1
    - timeout: as long as it has to be
- GoForwards:
    - use a suitable propeller demand
    - timeout: as long as it has to be
