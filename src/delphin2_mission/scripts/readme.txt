guideline for using state machine

for available behaviour check src/delphin2_mission/src/delphin2_mission/:
- basic_states.py
- construct_stateContainer.py

container types
- simple container:
    - to define an explicit transition between states
- concurrent container:
    - to execute multiple states simultaneously
- sequence container:
    - to execute a state in a sequence
- iteration container:
    - for recuring task
    
general meaning for state outcomes:
- succeeded: goal is fulfilled
- preempted: backSeatDriver flag is raised or the mission is terminated
- aborted: goal is not fulliled within timeout
