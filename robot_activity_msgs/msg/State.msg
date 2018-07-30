# Current state of a robot_activity
#
# Usually corresponds to a single node, but in case of super-nodes
# (similar to nodelets, multiple robot_activity can run in a
# single UNIX process) corresponds to a specific instance of robot_activity

uint8 INVALID=0       # State should never be INVALID

uint8 LAUNCHING=1     # during onCreate/onManagedCreate transition
uint8 UNCONFIGURED=2  # robot_activity is instantiated, but no yet configured
uint8 STOPPED=3       # robot_activity is stopped,
uint8 PAUSED=4        # robot_activity is paused, fast transition to running state
uint8 RUNNING=5       # robot_activity is running, fully operational state
uint8 TERMINATED=6    # robot_activity has been terminated

Header header         # frame_id field is irrelevant
string node_name      # full name of the robot_activity instance
uint8 state           # current state
