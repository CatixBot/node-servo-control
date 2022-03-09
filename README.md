# Servo node
## Description
This node provides functionality for each servo:
- Adjusting calibration by writing corresponding parameters
- Servo shaft angle rotation in radians by sending corresponding messages
## Messages
ServoState.msg:
- `servo_index` - Index of servo to rotate in clockwise notation
- `rotate_angle` - Rotation angle in radians
## Parameters
Each servo has the following list of parameters grouped by servo index:
- `first_point_angle` - First calibration point in radians
- `first_point_percentage` - First calibration point in signal strength percentage
- `lower_limit_percentage` - Minimum signal strength in percentage to prevent servo overloading
- `second_point_angle` - Second calibration point in radians
- `second_point_percentage` - Second calibration point in signal strength percentage
- `upper_limit_percentage` - Maximum signal strength in percentage to prevent servo overloading

_These parameters is required to build **linear calibration** and match servo signal strength in percentage and its angle in radians between two endpoints (first and second)_ 