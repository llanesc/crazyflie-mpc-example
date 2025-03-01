# crazyflie-mpc-example
This code is a model predictive path tracking control package for Crazyflie nano quadrotors that appears in our ICRA 2024 paper "CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor" [1]. We use [CrazySim](https://github.com/gtfactslab/CrazySim)
to simulate the drone behavior before deployment on real Crazyflie hardware.

![16cfs](https://github.com/user-attachments/assets/fd8eddd7-4de9-443c-9837-e7e3bd8157b4)

## References

[1] C. Llanes, Z. Kakish, K. Williams, and S. Coogan, “CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor,” To appear in 2024
IEEE International Conference on Robotics and Automation (ICRA), 2024.


```console
@INPROCEEDINGS{LlanesICRA2024,
  author={Llanes, Christian and Kakish, Zahi and Williams, Kyle and Coogan, Samuel},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor}, 
  year={2024},
  volume={},
  number={},
  pages={12248-12254},
  keywords={Sockets;Prediction algorithms;Hardware;Robustness;Sensors;Trajectory;Task analysis},
  doi={10.1109/ICRA57147.2024.10610906}}

```

# Setup

This section follows the setup of CrazySwarm2 with CrazySim and demonstrating the model predictive controller (MPC) in this code with [Acados](https://github.com/acados/acados) to track a set of predefined temporally parametrized trajectories.

Make sure to set up [CrazySim](https://github.com/gtfactslab/CrazySim) properly and follow the instructions through the Crazyswarm2 setup.

You now need to install Acados. Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

Then build the ROS 2 workspace.
```bash
cd ros2_ws
colcon build --symlink-install
```

### Configuration
The crazyswarm2  configuration files can be found in 
```bash
ros2_ws/src/crazyswarm2/crazyflie/config/
```
The crazyflies.yaml describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

The main code for the MPC script is in the following:
```bash
ros2_ws/crazyflie_mpc/crazyflie_mpc/crazyflie_multiagent_mpc.py
```
The trajectory type can be changed to a horizontal circle, vertical circle, helix, or a lemniscate trajectory by changing the variable "trajectory_type" in the CrazyflieMPC class. There is also a motors variable in the CrazyflieMPC class that can be changed based on if you defined the crazyflie or crazyflie_thrust_upgrade model.

### Start up the Firmware
Start up the firmware with any of the 3 launch script options. Below we demonstrate 4 Crazyflies in a square formation.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie
```

### Start Crazyswarm2
Make sure that `cf_1`, `cf_2`, `cf_3`, and `cf_4` are enabled in the CrazySwarm2 configuration YAML file. Launch the Crazyswarm2 services with CFLib backend.
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

### Start MPC code
### 
Run the Crazyflie MPC demonstration with the code below. The argument `n_agents` can be modified for the number of agents in your environment. Additionally, the argument `--build_acados` can be defined to compile the Acados optimal control problem.
```bash
ros2 run crazyflie_mpc crazyflie_multiagent_mpc --n_agents=4 --build_acados
```

Using the command line publisher we can command all vehicles to take off using MPC.
```bash
ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to start the trajectory.
```bash
ros2 topic pub -t 1 /all/mpc_trajectory std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to stop the trajectory and hover.
```bash
ros2 topic pub -t 1 /all/mpc_hover std_msgs/msg/Empty
```

We also implemented a MPC land feature, but it's still experimental and may result in crashing the drone.
