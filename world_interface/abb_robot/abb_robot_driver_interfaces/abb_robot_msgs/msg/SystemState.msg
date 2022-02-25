#===================================================================================================
# Description:
#   Message definition representing a subset of states of an ABB robot controller system.
#===================================================================================================

#---------------------------------------------------------------------------------------------------
# Message fields
#---------------------------------------------------------------------------------------------------

# Metadata.
std_msgs/Header header

# Motors on/off indicator.
bool motors_on

# Auto/manual mode indicator.
bool auto_mode

# RAPID running/stopped indicator.
bool rapid_running

# RAPID task states.
RAPIDTaskState[] rapid_tasks

# Mechanical unit states.
MechanicalUnitState[] mechanical_units
