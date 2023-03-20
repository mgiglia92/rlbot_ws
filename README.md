
># Installng/Building
>- Download steam, install rocket league.
>- Install ros2 and dev packages (do full)  
>- Need .venv in rlbot_bridge_pkg/lib/  
>- Activate, pip install rlbot  
>- colcon build  
>- Run . full_setup.bash will source python venv and ros env stuff  
>- ros2 run node rlbot_bridge_pkg AgentNode   
>- This will start rlbot and rocket league from steam  
>- ros2 run node simpler_controller_pkg SimplerControllerNode  
>- Open another terminal, use rlbot_bridge_pkg/testing_scripts srv_test.py to call the reset service on AgentNode.  
>- Run set_gains.py to set the gains on the controller. (Currently node defaults to zeros for pid gains)  

## Nodes
>**SimplerControllerNode**   
>- does a simple PID loop to control the linear and angular velocity of the car  
  
>**AgentNode**  
>- currently just sets throttle to Twist.linear.x and stter to Twist.angular.z  
  
>**TrajectoryGeneratorNode**  
>- Generate **optimal** trajectory, fit 3rd deg polynomial to it and send coefficients over trajectory topic  
  
>**PlotterNode**  
>- Plot data from various topics for analysis  

## Topics
>- /cmd_vel
>- /player0/RigidBodyTick
>- /current_trajectory

## Next Things
>- Add trajectory following controller (generate twist message from trajectory and err)
>- Add boost usage.