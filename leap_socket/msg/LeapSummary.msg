
# summary of data streamed from the leapmotion
# assumes a single hand

# How confident we are with a given hand pose [0.0, 1.0]
float32 confidence 

# time the hand has been tracked/visible (seconds)
float32 timeVisible 

# array of gestures sensed
string[] gestures 

# distance between the thumb and index finger of a pinch [0.0, 1.0]
float32 pinchStrength 

# strength of a grab pose, 1 is closed [0.0, 1.0]
float32 grabStrength

# unit direction vector of the palm position toward fingers 
geometry_msgs/Vector3 palmNormal

# center position of the palm from leap origin (mm)
geometry_msgs/Vector3 palmPosition

# rate of change of palm position (mm/second)
float32[] palmVelocity
