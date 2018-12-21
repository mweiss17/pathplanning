# Path planning under uncertainty - DuckieTown 2018 UdeM

## Installation
Prerequisites: ROS Kinetic or Melodic, Python 2.7

In `catkin_ws`, clone the package, then:

```$ catkin_make```

Source the setup file: ``` $ source ./devel/setup.bashrc```

And finally, if you are on Linux, you may need set the python files as executables:
```$ cd src/pathplan_uncertainty/src```
```$ sudo chmod +x dt_*```

## Launching full demo
The whole demo can be launched using: 

```$ roslaunch pathplan_uncertainty dt_pathplan.launch```

It launches together the simulator, the manager and the agent nodes.


## Simulator node

### Purpose
The simulator node allows to simulate the movement of two Duckiebots: our Duckiebot (controlled by the agent node) and the other Duckiebot (controlled by a pretermined policy). It allows to follow the position, orientation, safety status (collision?) and the type of ground over which our Duckiebot is currently (right lane, left lane, partially out of the road, completely out of the road).

### Launch
Can be launched using:

```$ roslaunch pathplan_uncertainty dt_simulator_node.launch```

### Parameters
Different parameters files are called by the simulator node.

In `sim.yaml`, you will find:



### Test
Can be tested using rqt to simulate an agent. Example here:

While node is running, in a new terminal, run `$ rqt`

From the top menu, open Plugins/Topics/Message Publisher.

From the developing menu, choose: 
`/agent/agent_command`, add it with a freq of 1 Hz.
Develop the topic and give a value of 5 to `compute_time_steps` (for example) and  `[0, 0, 0, 0, 0]` to `orientation_seq`.
Check the checkbox to start publishing.
The simulation should run with our DuckieBot having a 5 time-step straight trajectory.

### More details about communication

#### Listening to this topic:
..* ```/agent/command```: 
...Type: `pathplan_uncertainty/AgentCommand.msg`..
...Includes:..
... ..* ```computation_time_steps```_k_ time steps the agent will take to compute the next trajectory. ..

... ..* ```orientation_seq```: trajectory to be executed in the meantime (only orientation, in radians) ..

Once the simulator receives a message on this topic, it publishes the observation of current state, and starts executing the trajectory over the _k_ time steps.


#### Publishing these topics:
This topic should be listened to by the Agent:

..* ```/sim/obs/observations``` is the concatenation of both Duckies' poses, plus their velocities and their radius. It is sent **after the _k_ time steps have been executed**.

This topic should be listened to by the Manager and is published **at every time step**:

..* ```/sim/gt/world_state``` contains the time, reward, both Duckies' poses, our Duckie's safety status and the type of ground on which our Duckie is.

For debugging purposes, the node also publishes these topics **at every time step**:

..* ```/sim/gt/pose_our_duckie``` is the pose of the our Duckie.

..* ```/sim/gt/pose_other_duckie``` is the pose of the other Duckie.

..* ```/sim/gt/our_duckie_ground_type``` is the ground type on which our Duckie is.

..* ```/sim/gt/our_duckie_safety_status``` is the safety status of our Duckie.

The ground type and safety status are encoded according to the protocole defined in `/config/communcations.yaml`

For visualization purposes, the node also publishes this topic:

```/sim/road_image``` is an image showing the current state (more details in the *Seeing what is happening* section)

### Services
The ```get_ground_type``` service can be called to return the ground type on which a robot with a given position and radius would be.



## Manager Node
### Purpose
The manager node allows to track the evolution of the simulation. It records the state at every time step and computes the score.



## Agent Node
TO DO

## Seeing what is happening

You can see what is happening by launching ```$ rqt_image_view``` in a new terminal and follow the topic ```/sim/road_image```. The red circle represents our Duckiebot, driven by the agent. The yellow circle represents the other duckie bot, driving according to the policy of the ```other_duckie_type``` specified in the sim parameters. The dark line in each circle represent their current heading.