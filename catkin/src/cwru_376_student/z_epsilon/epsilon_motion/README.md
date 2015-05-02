# epsilon_motion


Here are example nodes that publish values on the "cmd_vel" topic.

The nodes are specialized to send to topic /cmd_vel, which works with physical robots (Jinx and Abby).

This version "dead_reck" is reactive.  It ramps velocity up and down and will recover
from halts.  


## Example usage
Can use this, e.g., with cwruBot Gazebo model or the physical robot.  With robot running, execute:
`roslaunch epsilon_motion epsilon_dead_launcher.launch`


    
