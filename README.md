# Giskard for Fetch

This package contains software that is neccessary to control the Fetch robot with Giskard.

## Joint Velocity Controllers
Giskard produces velocity commands for individual joints. This package provides a motor controller plugin that accepts commands in the form of a `sensor_msgs/JointState` and applies them to the robot's joints.

### Launching

The package provides the launch file *velocity_controllers.launch* to start the motor controller. This can be run on Fetch to load up the joint velocity controller.The controller can be configured using the *config/giskard_controllers.yaml* file. The possible parameters are:

```yaml
joints: []
watchdog_period: FLOAT
autostart: BOOL
```

 - `joints` is a list of the joints' names that the controller should control.
 - `watchdog_period` determines for how long a command is executed until it's deemed to old and the robot is stopped.
 - `autostart` determines whether the controller should start itself automatically after construction. 

**NOTE: Using the launch file will kill all other running motor controllers. These will also not be restarted after terminating the launched process.**

### Commanding
If you used the launch file, the controller will now be ready to recieve your commands on the topic `/velocity_controller/joint_velocity_controller/joint_velocity_commands`
The commands have to be supplied in the form of `sensor_msg/JointState` messages. These have to be correctly stamped, as their stamped is used to check against the watchdog timeout. The commands themselves are encoded in the `velocity' list of the message. The name of a joint in the `name` list and its corresponding command have to have the same index within their lists. Even though they're not used for the calculations, the `position` and `effort` lists have to be of the same length as the name list. A valid command to command torso, wrist and camera tilt could look like this:

```yaml
header: {...}
name: ['torso_lift_joint', 'wrist_roll_joint', 'head_tilt_join']
position: [0,0,0]
velocity: [0.2, -0.8, 0.13]
effort: [0,0,0]
```

Remember to publish commands frequently to achieve smooth robot motions and to avoid the joints being stopped by the watch dog. 

## Simulation
The package contains the `fetch_sim.launch` launch file that configures and starts an instance of the *iai_naive_kinematics_sim* to simulate a Fetch. The simulator can be cloned from https://github.com/code-iai/iai_naive_kinematics_sim.
