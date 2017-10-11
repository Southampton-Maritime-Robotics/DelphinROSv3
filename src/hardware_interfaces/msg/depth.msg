float32 depth         # depth as measured from sensor, in [m]
float32 depth_calib   # depth of the CoB, considering sensor distance and pitch angle, in [m]
float32 depth_filt    # depth_calib filtered by PT filter
float32 depth_der     # derivative of depth_calib from filtering
