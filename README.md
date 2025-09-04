# cartesian_impedance_control
### ROS2 cartesian_impedance_controller from Pd|Z

Prerequisites:
* ROS2 humble <br />
* Libfranka 0.13.0 or newer <br />
* franka_ros2 v0.13.1 <br />

For further information, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html)

Once you have everything set up, follow the steps below to get the controller running.

Clone this repository in the src directory of your franka_ros2_ws: <br />
```bash
cd franka_ros2_ws/src 
git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git
```
For the moment, you need to add the following lines of code, to your controllers.yaml file inside franka_ros2/franka_bringup/config/:
```bash
cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
```

Clone the messages package in the src directory: <br />
```bash
git clone https://github.com/CurdinDeplazes/messages_fr3.git
```

Build the package or whole workspace: <br />
```bash
colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

If not yet done, ensure your setup is always source by adding the following line to the end of your .bashrc file (to get access to it, you need to execute `nano .bashrc` in your home directory). : <br />
```bash
source /home/<user>/franka_ros2_ws/install/setup.sh 
```

Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Launch the client if you want to adjust parameters: <br />
``` bash
ros2 run cartesian_impedance_control user_input_client 
```

## Control Modes (Runtime Parameters)

Launch the Cartesian Impedance Controller (must be active before toggling any mode):
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

The controller (`CartesianImpedanceController`) exposes boolean parameters you can toggle at runtime (only ONE primary motion mode should be true at a time; internal logic enforces precedence):

| Mode | Parameter | Purpose | Core Topic(s) |
|------|-----------|---------|---------------|
| Cartesian Impedance (default) | (all false) | Standard stiffness / damping pose control (impedance + force logic) | `/franka/jacobian_ee` |
| Free Movement Mode | `free_movement_mode` | Gravity + damping (low resistance), no pose attraction | — |
| Policy Control Mode | `policy_control_mode` | Follows joint targets from learned RL / BC policy | `/policy_outputs` (Float64MultiArray, 7) |
| Trajectory Playback Mode | `trajectory_playback_mode` | Plays externally supplied joint sequences | `/trajectory_playback/joint_positions` |
| Reset To Default | `reset_to_default_position` | One‑shot filtered move back to nominal joint config | (internal) |
| Imitation Learning Mode | `imitation_learning_mode` | Cartesian pose tracking from IL policy / teleop | `/cartesian_position_controller/commands` (Float64MultiArray: 7) |

Precedence (highest first): imitation_learning_mode > trajectory_playback_mode > policy_control_mode > free_movement_mode. `reset_to_default_position` overlays free mode.

### Enable / Disable Modes

(Controller node: `/cartesian_impedance_controller`)
```bash
# Inspect parameters
ros2 param list /cartesian_impedance_controller

# Enable free movement
ros2 param set /cartesian_impedance_controller free_movement_mode true

# Switch to policy control
ros2 param set /cartesian_impedance_controller free_movement_mode false
ros2 param set /cartesian_impedance_controller policy_control_mode true

# Start trajectory playback
ros2 param set /cartesian_impedance_controller policy_control_mode false
ros2 param set /cartesian_impedance_controller trajectory_playback_mode true

# Trigger reset (auto-clears)
ros2 param set /cartesian_impedance_controller reset_to_default_position true

# Enable imitation learning
ros2 param set /cartesian_impedance_controller imitation_learning_mode true
```
