#===================================================================================================
# Description:
#   Message definition representing the runtime states of all known
#   RobotWare "StateMachine Add-In" RAPID instances.
#===================================================================================================

#---------------------------------------------------------------------------------------------------
# Message fields
#---------------------------------------------------------------------------------------------------

# Metadata.
std_msgs/Header header

# "StateMachine Add-In" instances.
StateMachineState[] state_machines
