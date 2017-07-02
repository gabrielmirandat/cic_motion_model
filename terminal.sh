#!/bin/bash
echo "terminal.."

echo "configuring top workspace.."
. ~/ros/projects_ws/devel/setup.bash

echo "launching.."
rosrun cic_motion_model cic_motion_model_node
