# Feedback message from the tail section
float32 b_sp  # setpoint currently sent to arduino
float32 b_ack # setpoint acknowledgement by arduino, -2 if nothing received from arduino
float32 b_fb  # actual measured actuator state, -2 if nothing received from arduino
float32 c_sp
float32 c_ack
float32 c_fb
float32 d_sp
float32 d_ack
float32 d_fb
float32 e_sp
float32 e_ack
float32 e_fb
float32 prop_sp
float32 prop_ack
float32 prop_rps
