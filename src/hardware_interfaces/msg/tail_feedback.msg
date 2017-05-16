# Feedback message from the tail section
# each of the actuators publishes 3 values:
# _sp : setpoint currently sent to arduino
# _ack : current output to the sensor from the arduino; 
# this value is more for relaitve change rather than quantitative information
# _fb : feedback measurement made by the arduino
float32 b_sp  # in [degrees]
float32 b_ack
float32 b_fb  # in [degrees]
float32 c_sp
float32 c_ack
float32 c_fb
float32 d_sp
float32 d_ack
float32 d_fb
float32 e_sp
float32 e_ack
float32 e_fb
float32 prop_sp  # in [rpm]
float32 prop_ack
float32 prop_rps # in [rpm]
