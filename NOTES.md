# Rocket League Input analysis
- Throttle input:
  - \> 0.01181 causes acceleration
  - 0 < 0.01181 causes deceleration 
  - < 0 regular braking curve

# ros2 service cmd line
ros2 service call /simple_controller/twist_setpoint rlbot_msgs/srv/TwistSetpoint '{setpoint: {linear: {x: 100, y: 0, z: 0}, angular: {z: 1}}}'