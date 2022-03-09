# Servo node
## Description
This node provides functionality for each servo:
- Servo shaft angle rotation in radians by sending corresponding messages
- Adjusting calibration by writing corresponding parameters
## Messages
ServoState.msg:
- `servo_index` - Index of servo to rotate, which determined by joint indexing in kinematic scheme
- `rotate_angle` - Shaft angle rotation in radians, which values determined by calibration
## Parameters
Each servo has the following list of parameters grouped by servo index:
- `first_point_angle` - First calibration point in radians
- `first_point_percentage` - First calibration point in percentage of signal strength
- `lower_limit_percentage` - Minimum in percentage of signal strength to prevent servo overloading
- `second_point_angle` - Second calibration point in radians
- `second_point_percentage` - Second calibration point in percentage of signal strength
- `upper_limit_percentage` - Maximum in percentage of signal strength to prevent servo overloading

_These parameters is required to build **linear calibration** determines relationship between servo signal strength in percentage and its angle in radians for each servo_