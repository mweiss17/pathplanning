# Path planning under uncertainty - DuckieTown 2018 UdeM

## Installation
Prerequisites: ROS Kinetic, Lunar or Melodic, Python 2.7, catkin, scipy

In `catkin_ws/src`, clone the package, then:

```$ catkin_make```

Source the setup file: ``` $ source ./devel/setup.bashrc```

And finally, if you are on Linux, you may need set the python files as executables:
```$ cd src/pathplan_uncertainty/src```
```$ sudo chmod +x dt_*```

## Launching full demo
The whole demo can be launched via the terminal:

```roslaunch pathplan_uncertainty dt_pathplan.launch```

It launches the simulator, the manager, and the agent nodes together.


## Simulator node

### Purpose
The simulator node allows to simulate the movement of two Duckiebots: our Duckiebot (controlled by the agent node) and the other Duckiebot (controlled by a pre-determined stochastic policy). It has perfect information about our current position, orientation, safety status (i.e. was there a collision) and lane position (right lane, left lane, partially out of the road, completely out of the road). N.B. that we use lane position and ground_type synonymously.

### Launch
The simulator ros node can be launched on its own with this command:

```roslaunch pathplan_uncertainty dt_simulator_node.launch```

### Test
Can be tested using rqt to simulate an agent. Example here:

While node is running, in a new terminal, run `$ rqt`

From the top menu, open Plugins/Topics/Message Publisher.

From the developing menu, choose:
`/agent/agent_command`, add it with a freq of 1 Hz.
Develop the topic and give a value of 5 to `compute_time_steps` (for example) and  `[0, 0, 0, 0, 0]` to `orientation_seq`.
Check the checkbox to start publishing.
The simulation should run with our DuckieBot having a 5 time-step straight trajectory.

### Communication

#### Listening to this topic:
  * ```/agent/command```, including:
    * ```computation_time_steps```: _k_ time steps the agent will take to compute the next trajectory.

    * ```orientation_seq```: trajectory to be executed in the meantime (only orientation, in radians).

Once the Simulator receives a message on this topic, it publishes an observation and starts executing the trajectory over the _k_ time steps.


#### Publishing on these topics:
This topic should be listened to by the Agent:

  * ```/sim/obs/observations``` includes both Duckies' poses, their velocities, and their radii. It is sent **after the _k_ time steps have been executed**.

This topic should be listened to by the Manager and is published **at every time step**:

  * ```/sim/gt/world_state``` contains the time, both Duckies' poses, our Duckie's safety status and the type of ground on which our Duckie is.

For debugging purposes, the node also publishes these topics **at every time step**:

  * ```/sim/gt/pose_our_duckie``` is the pose of the our Duckie.

  * ```/sim/gt/pose_other_duckie``` is the pose of the other Duckie.

  * ```/sim/gt/our_duckie_ground_type``` is the ground type on which our Duckie is.

  * ```/sim/gt/our_duckie_safety_status``` is the safety status of our Duckie.

The ground type and safety status are encoded according to the protocole defined in `/config/communcations.yaml`

For visualization purposes, the node also publishes this topic:

```/sim/road_image``` is an image showing the current state (more details in the *Seeing what is happening* section)

#### Service
The ```get_ground_type``` service can be called to return the ground type on which a robot with a given position and radius would be.

### Technical details

#### Coordinate system
We use a fixed coordinate system (it does not follow our Duckiebot).
  * *x* is the lateral coordinate, raising from left to right. It is 0 at the center of the right lane. It is in meters.
  * *y* is the vertical coordinate., raising from down to up. It is 0 at the initial position of our Duckiebot. It is in meters.
  * *theta* is the angular coordinate. It is 0 when looking up, *-pi*/2 when looking left, *pi* when looking down and yes, * pi/2* when looking right. Yes, we know, it is the other way around than the usual definition. It was made "on purpose" to reward those who read the instructions completely. It is in radians.

#### How is our Duckiebot movement encoded?

When the `orientation_seq` message arrives, it contains a sequence of orientations in radians. We consider that our Duckiebot has a constant speed and cannot accelerate nor break. Therefore, at each time step, it will follow the orientation given by `orientation_seq`. 
Note that the Duckiebot cannot turn extremely fast. Therefore, we limit its orientation change at every time step: if the difference between the orientation command and the current orientation is bigger than the limit, the orientation change is set to the limit.

#### How is the other Duckiebot movement working?

The other Duckiebot is going straight forward. Remember: it is a truck and the sleepy driver does not touch the wheel. However, the driver plays with the accelerator and the breaks. We model this by drawing, at every time step, an acceleration from an uniform distribution. The truck is not unlimited though: it has a minimum and a maximal velocity, from which it cannot respectively break or accelerate anymore.

## Manager Node
### Purpose
The Manager tracks the evolution of the simulation. It records the state at every time step and computes the score.

### Launch
Can be launched using:

```$ roslaunch pathplan_uncertainty dt_manager_node.launch```

### Communication

#### Listening to this topic:
  * `/sim/gt/world_state` contains the time, both Duckies' poses, our Duckie's safety status and the type of ground on which our Duckie is.

Once the Manager receives this message, it records the data and computes the reward and score of the Duckiebot. Then, it publishes the score.

#### Publishing on this topic:
  * `/manager/current_score` is the current score of the Duckiebot, published at every time step.

### Service
The `/manager/get_manager_records` service can be called to return the whole record of states.


## Agent Node
### Purpose
The Agent receives observations from the simulator and controls the actions of our Duckiebot. Because this whole program works in discrete time steps, the Agent reports its computation time to the Simulator and incurs a lag before updated path plans take effect on our duckiebot.

### Launch
Can be launched using:
``` $ roslaunch pathplan_uncertainty dt_agent_node.launch```

### Communications

#### Listening to this topic:
  * `/sim/obs/observations` : includes both Duckies' poses, their velocities, and their radii.

Once the Agent receives this message, it computes the best trajectory, simulates its computation time, and publishes them.

#### Publishing on this topic:
  * `/agent/command`, including:
    * `computation_time_steps`: _k_ time steps the agent will take to compute the next trajectory.

    * `orientation_seq`: trajectory to be executed in the meantime (only orientation, in radians).

### Computation of the trajectory
In our case, the computation of the best trajectory is done in two parts:

1. Using the observations and a known movement model of the other Duckiebot, the Agent predicts the probability of collision at each time step for any x, y position of our Duckiebot. This is done in two steps:
  a. Given the position, heading and velocity of the other Duckiebot, the Predictor uses its probabilistic velocity model to approximate the probability of the other Duckiebot's position (in the *y* axis only) at every time step until a given horizon. This is saved on a discretized map.
  b. Then, given a x, y position for a time step, the Predictor checks every *y* point on the discretized map that would lead into a collision, and add the probabilities of the other Duckiebot to be there to give the probability of collision.
2. Using Monte Carlo Tree Search with orientation change as the unique parameter, the agent finds the path with the highest reward (or lowest negative reward).

## Timing

You will have noticed that the nodes do not run in real time. Instead, it is a simple loop.

### How does it work?
1. To kickstart the process, the Agent sends an empty command with *k = 1* computation time step.
2. The Simulator sends the observations to the Agent and then propagates the world for *k = 1* time step with an empty command (our Duckiebot does not move).
3. The Agent receives the observations. It computes the next trajectory and sends it with a random computation time step number *k*.
4. The Simulator receives the trajectory and *k*. It sends the observations to the Agent and then propagates the world for *k* time steps with the given trajectory.
5. The Agent receives the observations. Repeat stage 3, then 4, until program is stopped using Ctrl+C

At every time step, the Simulator also publishes the ground truth state so that the Manager can record the trajectory and count the score. It also publishes the image so that we humans can visualize what is happening. In order for the image sequence to make sense, the Simulator waits a bit after having published each image.

### Why did we do it like this?
The computation cost of the optimal trajectory can be heavy. It may not be realistic to expect it to be done at every time step. In the real world, as soon as the Agent is done computing, it sends it to the actuators, observes the world, and starts computing again with the new informations. This is the way it is modeled in this case.

## Seeing what is happening

You can see what is happening by launching ```$ rqt_image_view``` in a new terminal and follow the topic ```/sim/road_image```. The red circle represents our Duckiebot, driven by the agent. The yellow circle represents the other duckie bot, driving according to the policy of the ```other_duckie_type``` specified in the sim parameters. The dark line in each circle represent their current heading. The white curve represents the trajectory that our Duckiebot has decided to follow for the next few time steps.




## Parameters
Different parameters files are called by the different nodes. They can be found in the `/config/` directory.

In `sim.yaml`, you will find:
  * `dt` : the value in seconds of each time step in the simulation
  * `dt_in_sim` : the value in seconds during which the simulation waits at each time step (allows the video to be real time)
  * `image`: parameters of the image, including its height, width, meter to pixel ratio, and the baseline in pixels from which y=0 is displayed, from the bottom of the image. You can disable the image output by setting `output_image` to False.
  * `world`: world parameters. Mainly, the width of the whold road, in meters (each lane is therefore half of the road width).
  * `other_duckie_type` : the type of the other Duckiebot. Can be set to `constant_speed_duckie` or to `unstable_speed_duckie`

In `duckiebots.yaml`, you will find, for each Duckiebot type:
  * `start_pose` : the pose at start (x, y, theta) with x, y in meters, x = 0 the center of the right lane, and theta in radians, theta = 0 looking forward.
  * `velocity` : the velocity at start, in m/s
  * `radius` : the radius of the Duckiebot, in meters - we consider the Duckiebot to be a circle
  * `type` : the type of the Duckiebot
  * `angle_change_limit` : the maximal change in orientation between two time steps, in radians
  * `max_acceleration` : the maximal acceleration in a second that the Duckiebot can undertakȩ
  * `max_velocity` and `min_velocity` : the maximal and minimal velocities of the Duckiebot, in m/s

In `rewards.yaml`, you will find the rewards given at each time step:
  * `status_fine` : while there is no collision
  * `status_collision` : when there is a collision
  * `type_right_lane`: when the Duckiebot is in the right lane
  * `type_wrong_lane`: when the Duckiebot is in the wrong lane
  * `type_partially_out`: when the Duckiebot is partially out of the road
  * `type_lost`: when the Duckiebot is completely out of the road

In `communications.yaml`, you will find the communication protocole used by the nodes to exchange information on the safety status and the ground type on which the Duckiebot is.

In `agent.yaml`, you will find the parameters used by the agent to predict, plan, and simulate the computation time:
  * `predictor` :
    * `time_horizon` : time in seconds until which the predictor predicts the position probability of the other Duckiebot
    * `y_resolution` : resolution in meters of the discretization of the y dimension for the predictor
    * `y_horizon` : distance in meters from the origin until which the predictor runs
    * `vel_resolution` : resolution in m/s of the velocity change discretization. For example, with `dt` = 0.2s, and an uniformly distributed acceleration between -`max_acceleration` and + `max_acceleration` with `max_acceleration` = 2 m/s^2, the predictor will at each time step consider the possible velocity changes being of [-0.4, -0.2, 0, 0.2, 0.4] m/s, each of them with a 0.2 probability.
  * `computation_time` :
    * `mean` : average number of time steps taken to simulate the computation time
    * `std_dev` : standard deviation in time steps of the simulated computation time
  * `mcts` :
    * `scalar` : raise this number to increase the exploration behavior, lower the number to increase the exploitation behavior of the MCTS
    * `budget` : Iterations of the MCTS through the tree. In each iteration, a new node is added so the tree is expanded.
    * `time_steps` : amount of time steps the MCTS is looking ahead
