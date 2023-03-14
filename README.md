Install ros2 and dev packages (do full)  
Need .venv in rlbot_bridge_pkg/lib/  
Activate, pip install rlbot  
colcon build  
Run . full_setup.bash will source python venv and ros env stuff  
ros2 run node rlbot_bridge_pkg AgentNode   
This will start rlbot and rocket league from steam  
ros2 run node simpler_controller_pkg SimplerControllerNode  
This will subscribe to the game tick packet and currently do a crude velocity pid control to the agent by publishing to cmd_vel  
Agent currently just sets throttle to Twist.linear.x and stter to Twist.angular.z  
