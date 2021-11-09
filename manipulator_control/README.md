# Manipulator Control Package
Maintained by Sarah Ziselman for graduate independent project at Northwestern University.

## Overview
This package uses `MoveIt`'s open-source motion planner to plan and execute commands on the HDT Adroit Manipulator Arm. It's primary function is to return the time that it takes to execute moving a specified block out of the scene.

Nodes:
* `manipulator_control`
* `fake_manipulator_blocks`

Services:
* `TrajectoryExecution`
* `RemoveObject`

## Testing
In order to test the `manipulator_control` node, the `fake_manipulator_blocks` node was created to send fake information and verify that it returns the correct execution time. In order to launch the two nodes, use the following command:
```
roslaunch manipulator_control fake_blocks.launch
```

## Usage

## Configuration