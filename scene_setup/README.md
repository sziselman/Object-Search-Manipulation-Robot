# Scene Setup Package
Maintained by Sarah Ziselman for graduate independent project at Northwestern University.

## Overview
This package takes sets up the scene and returns that information to other nodes that may need it. It's primary function is to return the volume of the occluded space caused by a block within a scene.

Nodes:
* `scene_setup`
* `fake_blocks`

Libraries:
* `scene_geometry_lib`

Services:
* `Visibility`

## Testing
In order to test the `scene_setup` node, the `fake_blocks` node was created to send fake information and verify that it returned the correct visibility. In order to launch the two nodes, use the following command:
```
roslaunch scene_setup fake_blocks.launch
```

## Usage

## Configuration