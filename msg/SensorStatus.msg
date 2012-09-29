# This message holds the status of an individual component of the robot.
# Based on Diagnostic messages.

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
float32 value # Value associated with the status
float32 min
float32 max
float32 nominal
