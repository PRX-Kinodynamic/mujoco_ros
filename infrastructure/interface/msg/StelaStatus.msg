# Stela has multiple threads, only some status are thread-specific

# Common
int16 IDLE=0
int16 INITIALIZING=1
int16 ERROR=100

# Replan
int16 REPLANNING=2
int16 VALIDATING=3

# STELA / FG 
int16 ADDING_TREE=4
int16 GRAPH_UPDATE=5 # Addition & deletion of nodes => sliding the window
int16 OBSERVATION=6 
int16 UPDATE_ESTIMATION=7
int16 PUBLISH_CONTROL=8

std_msgs/Header header
int16 state
