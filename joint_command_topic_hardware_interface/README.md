# Joint Command Topic Based System

The Joint Command Topic Based System implements a ros2_control `hardware_interface::SystemInterface` supporting command and state interfaces through the ROS topic communication layer. It publishes commands as `control_msgs/JointCommand` messages, one message per interface type (position, velocity, effort), on a dedicated topic per interface type.

## ros2_control urdf tag

The `joint_command_topic_hardware_interface` has a few `ros2_control` urdf tags to customize its behavior.

### Parameters

* joint_commands_topic: (default: "/robot_joint_commands"). Base topic for the joint command topics. Example: `<param name="joint_commands_topic">/my_topic_joint_commands</param>`.
* joint_states_topic: (default: "/robot_joint_states"). Example: `<param name="joint_states_topic">/my_topic_joint_states</param>`.
* trigger_joint_command_threshold: (default: 1e-5). Used to avoid spamming the joint command topic when the difference between the current joint state and the joint command is smaller than this value, set to -1 to always send the joint command. Example: `<param name="trigger_joint_command_threshold">0.001</param>`.
* sum_wrapped_joint_states: (default: "false"). Used to track the total rotation for joint states the values reported on the `joint_commands_topic` wrap from 2*pi to -2*pi when rotating in the positive direction. (Isaac Sim only reports joint states from 2*pi to -2*pi) Example: `<param name="sum_wrapped_joint_states">true</param>`.

### Per-joint Parameters

* mimic: Defined name of the joint to mimic. This is often used concept with parallel grippers. Example: `<param name="mimic">joint1</param>`.
* multiplier: Multiplier of values for mimicking joint defined in mimic parameter. Example: `<param name="multiplier">-2</param>`.

## Example

```xml
        <ros2_control name="name" type="system">
            <hardware>
              <plugin>joint_command_topic_hardware_interface/JointCommandTopicSystem</plugin>
              <param name="joint_commands_topic">/topic_based_joint_commands</param>
              <param name="joint_states_topic">/topic_based_joint_states</param>
            </hardware>
            <joint name="joint_1">
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
        </ros2_control>
```

## Topics

For each `write()` call, one `control_msgs/JointCommand` message is published per interface type to a topic derived from the `joint_commands_topic` parameter:

* `<joint_commands_topic>/position`: contains the joints that expose a `position` command interface.
* `<joint_commands_topic>/velocity`: contains the joints that expose a `velocity` command interface.
* `<joint_commands_topic>/effort`: contains the joints that expose an `effort` command interface.

Each message carries the joints of that interface type in its `joint_names` field, the matching command values in `values`, and the interface type in `interface_name`. Subscribers must listen on the topic that matches the interface they are interested in, e.g. with the default parameter a joint with `position` and `velocity` command interfaces publishes to `/robot_joint_commands/position` and `/robot_joint_commands/velocity` every cycle.
