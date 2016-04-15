# AUV status
float32 heading
float32 yawRate
float32 forwardVel
float32 swayVel

# error parameters
float32 heading_demand
float32 error

# controller terms
float32 N_Pterm
float32 N_Iterm
float32 N_Dterm

# actuator demand
int32 CS_demand
int32 thruster0
int32 thruster1
int32 controller_onOff
