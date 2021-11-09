# Greedy Search Package
Maintained by Sarah Ziselman for graduate independent project at Northwestern University.

## Overview
This package implements greedy search on a scene and visibile objects within it. It takes in information from the `scene_setup` node and the `manipulator_control` node. 

Nodes:
* `greedy_search`

Libraries:
* `greedy_search_lib`

## Testing
In order to test the `greedy_search` node, the `fake_search` node was created to send fake information and verify that the correct arrangement was returned. Use the following commands:
```
roslaunch greedy_search fake_search.launch
```

## Usage

## Configuration