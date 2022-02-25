#===================================================================================================
# Description:
#   Message definition representing the success/failure result of a service,
#   with different result codes and messages for corresponding outcomes.
#
# Note:
#   Result codes are 16 bit unsigned.
#===================================================================================================

#---------------------------------------------------------------------------------------------------
# Constant fields
#---------------------------------------------------------------------------------------------------

# Generic OK.
uint16 CODE_SUCCESS=1
string MESSAGE_SUCCESS=""

# Generic failure message.
uint16 CODE_FAILED=2
string MESSAGE_FAILED="Service failed (e.g. timeout or resource not found)"

# The server's underlying communication interface is busy.
uint16 CODE_SERVER_IS_BUSY=1001
string MESSAGE_SERVER_IS_BUSY="Server is busy"

# System state is missing.
uint16 CODE_SYSTEM_STATES_MISSING=1002
string MESSAGE_SYSTEM_STATES_MISSING="No system states received"

# Filename field is empty in service request.
uint16 CODE_EMPTY_FILENAME=2001
string MESSAGE_EMPTY_FILENAME="Filename is empty"

# Signal name field is empty in service request.
uint16 CODE_EMPTY_SIGNAL_NAME=2002
string MESSAGE_EMPTY_SIGNAL_NAME="Signal name is empty"

# RAPID task name field is empty in service request.
uint16 CODE_EMPTY_RAPID_TASK_NAME=2003
string MESSAGE_EMPTY_RAPID_TASK_NAME="RAPID task name is empty"

# RAPID module name field is empty in service request.
uint16 CODE_EMPTY_RAPID_MODULE_NAME=2004
string MESSAGE_EMPTY_RAPID_MODULE_NAME="RAPID module name is empty"

# RAPID symbol name field is empty in service request.
uint16 CODE_EMPTY_RAPID_SYMBOL_NAME=2005
string MESSAGE_EMPTY_RAPID_SYMBOL_NAME="RAPID symbol name is empty"

# Robot controller is not in auto mode.
uint16 CODE_NOT_IN_AUTO_MODE=3001
string MESSAGE_NOT_IN_AUTO_MODE="Robot controller is not in AUTO mode"

# The motors are off.
uint16 CODE_MOTORS_ARE_OFF=3002
string MESSAGE_MOTORS_ARE_OFF="Motors are off"

# The motors are on.
uint16 CODE_MOTORS_ARE_ON=3003
string MESSAGE_MOTORS_ARE_ON="Motors are on"

# RAPID has not been stopped.
uint16 CODE_RAPID_NOT_STOPPED=3004
string MESSAGE_RAPID_NOT_STOPPED="RAPID has not been stopped"

# RAPID is not running.
uint16 CODE_RAPID_NOT_RUNNING=3005
string MESSAGE_RAPID_NOT_RUNNING="RAPID is not running"

# Mechanical unit does not exist.
uint16 CODE_UNKNOWN_MECHANICAL_UNIT=3006
string MESSAGE_MECHANICAL_UNIT="Mechanical unit does not exist"

# RobotWare StateMachine Add-In runtime state is missing.
uint16 CODE_SM_RUNTIME_STATES_MISSING=4001
string MESSAGE_SM_RUNTIME_STATES_MISSING="No runtime states received for any StateMachine Add-In instance"

# RobotWare StateMachine Add-In-instances do not know RAPID task.
uint16 CODE_SM_UNKNOWN_RAPID_TASK=4002
string MESSAGE_SM_UNKNOWN_RAPID_TASK="RAPID task not found among known StateMachine Add-In instances"

# RobotWare StateMachine Add-In instance has not been initialized.
uint16 CODE_SM_UNINITIALIZED=4003
string MESSAGE_SM_UNINITIALIZED="StateMachine Add-In instance has not been initialized"

#---------------------------------------------------------------------------------------------------
# Message fields
#---------------------------------------------------------------------------------------------------

# Result code indicating service success/failure.
uint16 code

# Result message complementing the result code
#
# Note: Empty if service succeeded.
string message
