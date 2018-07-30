# Error emitted by robot_activity
#
# robot_activity can emit errors during processing, or the user implementing
# the robot_activity can emit error himself

uint8 UNEXPECTED_PROCESS_STOP=0
uint8 FROZEN_NODE=1
uint8 INVALID_INPUT_DATA=2       # invalid data e.g. file does not exist
uint8 RECOVERABLE_ERROR=3        # error is recoverable e.g.
uint8 FATAL_ERROR=4              # error implies node shutdown

Header header           # frame_id field is irrelevant
string node_name        # full name of the robot_activity instance
string function         # function where the error has occurred
string description      # description of error
uint8 error_type        # error type
