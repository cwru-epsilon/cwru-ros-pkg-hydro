# epsilon_steering_algorithm

This example code is NOT COMPLETE.  It will compile and run, but it does not contain the control theory for steering.

It does contain necessary hooks.  It subscribes to desState (a stream of desired states, including pose and twist).
It also subscribes to odom.  It computes a path lateral offset error and a path heading error.  These errors are published on
topic "steering_errs", and these values can be plotted with rqt_plot.

## Example usage
Start up Jinx or cwruBot

Start the steering algorithm with:
`rosrun epsilon_steering_algorithm epsilon_steering_algorithm'

Start up desired state generation:
`rosrun epsilon_des_state_generator epsilon_des_state_generator`

Give the desired state generator a path, e.g. by running:
`rosrun epsilon_des_state_generator epsilon_path_sender`



## Running tests/demos
    
