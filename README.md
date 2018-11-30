# Path planning under uncertainty - DuckieTown 2018 UdeM

## Installation
In `catkin_ws`, clone the package, then:

```$ catkin_make```

Finally: ``` $ source ./devel/setup.bashrc```


## Simulator node

### Launch
Can be launched using:

```$ roslaunch pathplan_uncertainty dt_simulator_node.launch```

### Parameters
You can change the simulations parameters such as the time step duration `dt` or the start pose, radius and type of each bot in the `config/sim.yaml` file.

Warning: for now, the only type implemented for the other duckie is the SlowDuckie.
Warning: for now, the parameter "velocity" is not used and the velocity is hard-coded.

### Test
Can be tested using rqt to simulate an agent. Example here:

While node is running, in a new terminal, run `$ rqt`

From the top menu, open Plugins/Topics/Message Publisher.

From the developing menu, choose: 
`/agent/computation_time_steps`, add it with a freq of 1 Hz, and give a value of 5 in the data (for example).

From the developing menu, choose: `/agent/orientation_seq`, add it with a freq of 1 Hz, and give the value of `[0, 0, 0, 0, 0]`

The simulation should run with our DuckieBot having a 5 time-step straight trajectory.

### More details about communication

#### Listening to these topics:
These two topics should be published by the Agent:

```/agent/computation_time_steps```: _k_ time steps the agent will take to compute the next trajectory

```/agent/orientation_seq``` : trajectory to be executed in the meantime (only orientation, in radians)

Once the simulator receives these two topics, it sends the observed pose of current state, and starts executing the trajectory over the _k_ time steps.

More topics should come soon linked to the Manager.

#### Publishing these topics:
These two topics should be listened to by the Agent:

```/sim/obs/pose_our_duckie``` is the pose of our Duckie observed **after all _k_ time steps** have been executed.

```/sim/obs/pose_other_duckie``` is the pose of the other Duckie observed **after all _k_ time steps** have been executed.

These four topics should be listened to by the Manager:

```/sim/gt/pose_our_duckie``` is the pose of the our Duckie published **at every time step**.

```/sim/gt/pose_other_duckie``` is the pose of the other Duckie published **at every time step**

```/sim/gt/our_duckie_ground_type``` is the ground type on which our Duckie is, published at every time step.

```/sim/gt/our_duckie_safety_status``` is the safety status of our Duckie, published at every time step.



## Manager Node
TO DO

## Agent Node
TO DO
