# mission launch file will run ros nodes and set the parameters in the parameter server.
# load set_auv_system when wanting to test on the actual AUV
# load set_auv_system_dummy when wanting to test on the auv simulator

myMission.launch
- system_loader.txt
-- set_auv_system.txt (interchangable with 2)
-- set_auv_system_dummy.txt (interchangable with 1)
-- default_parameters.yaml


