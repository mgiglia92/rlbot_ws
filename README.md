
# Installng/Building
>- Download steam, install rocket league.
>- Install ros2 and dev packages (do full)  
>- Make venv in root of ws
>- Activate, pip install rlbot, pyquaternion, scipy
>- Install setup/python_req.txt for venv  
>- colcon build  in root of ws
>- TODO:Make full_setup.bash non-absolute
>- Run . full_setup.bash will source python venv and ros env stuff  (i recommend adding it to your .bashrc file that way all debugger terminals and regular terminals are already sourced and ready to go, i haven't found a way to specify running a command before opening a debuggin terminal in vscode yet)
>- Run steam BEFORE running AgentNode that way the agentnode instance isn't spawning the steam app
>- ros2 run node rlbot_bridge_pkg AgentNode   (you'll have to confirm in the steam app that you want to run rocketleague with the extra parameters)
>- This will start rlbot and rocket league from steam  
>- ros2 launch rlbot_bridge_pkg dev_launch.py (will launch whatever dev setup was last committed, currently its' just setting the gains for the simpleControllerNode.)
>- Run the debugger for all the other nodes to complete the topic loop. ReferenceGeneratorNode, StanleyControllerNode and SimpleControllerNode will need to be run. You can also run auto_reset.py to make the system restart every 15 seconds good for debugging trajectory tracking.

## Nodes
>**SimplerControllerNode**   
>- does a simple PID loop to control the linear and angular velocity of the car
  
>**AgentNode**  
>- currently just sets throttle to Twist.linear.x and steer to Twist.angular.z  
  
>**TrajectoryGeneratorNode**  
>- Generate **optimal** trajectory, fit 3rd deg polynomial to it and send coefficients over trajectory topic  

>**ReferenceGeneratorNode**
>- This Node currently just finds the minimum distance between the vehicle and a predefined half circle and publishes the position and velocity found by the minimizer

>**StanleyControllerNode**
>- An implementation of the stanley geometric controller. Becuase rocket leagues steering input is [-1,1] and not an actual steering angle, some care needs to be taken here in regards to scaling of the input to the controller. The change in minimum turing radius as a function of velocity also makes this tough to tune across all velocities.
  
>**PlotterNode**  
>- Plot data from various topics for analysis  

## Topics
>- /cmd_vel
>- /controller_reference
>- /internals_stanle
>- /player0/RigidBodyTick
>- /simple_controller/internals
>- /trajectory_reference

## Next Things
>- Add trajectory following controller (generate twist message from trajectory and err)
>- Add boost usage.
