# The elapsed duration of the recognized movement up to the frame containing this 
# Gesture object, in microseconds. 
int64 duration_us

# The elapsed duration in seconds.
float32 duration_s

# gesture type "circle" "swipe" "screentap" "keytap"
string type

# current state of the gesture "start" "update" "stop"(start, update, swipe)
string state

# A string containing a brief, human-readable description of this Gesture. 
string to_string

float32[] center 
float32[] direction
int32[] handIds
int32 id

# The list of fingers, tools ids associated with this Gesture, if any. 
int32[] pointableIds 

# position of gesture (swipe, keytap, screentap)
float32 position

# progress of gesture (circle, keytap, screentap)
float32 progress 

# size of the circle gesture (circle)
float32 radius

# speed of gesture in mm/s (swipe)
float32 speed

# beginning position of swipe gesture (swipe)
float32[] startPosition
