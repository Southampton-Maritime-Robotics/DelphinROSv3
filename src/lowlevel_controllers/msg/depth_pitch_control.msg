###################
## depth parameters

float32 depth
float32 depth_demand
float32 error_depth
float32 int_error_depth
float32 der_error_depth

float32 deadzone_Depth

float32 Depth_Pgain
float32 Depth_Igain
float32 Depth_Dgain
float32 Depth_Pterm
float32 Depth_Iterm
float32 Depth_Dterm
float32 Depth_Thrust

float32 pitchBias
float32 pitchBiasMax
float32 pitchBiasGain_P # derivative term is at the bottom

###################
## pitch parameters
float32 pitch
float32 pitch_demand
float32 error_pitch
float32 int_error_pitch
float32 der_error_pitch

float32 deadzone_Pitch

float32 Pitch_Pgain
float32 Pitch_Igain
float32 Pitch_Dgain
float32 Pitch_Pterm
float32 Pitch_Iterm
float32 Pitch_Dterm
float32 cr
float32 crMax

#############################
## thruster parameters
float32 Thrust_Smax
int32 thruster0
int32 thruster1

###################
## control surfaces
int8 CS_Smax
float32 CS_Pgain
float32 CS_Igain
float32 CS_Dgain
float32 CS_Pterm
float32 CS_Iterm
float32 CS_Dterm
int32 CS_demand

float32 pitchBiasGain_D
