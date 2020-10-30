# kinematic-chain-uncertainty

This set of tools models uncertainty in joint angles and link length for a
kinematic chain. All tools are written in MATLAB.


## Experiments

### prismatic_arm_gaussian.m
This file produces a cloud of _exact_ possible endpoints for a kinematic chain
given configuration and uncertainty.

### prismatic_arm_jacobian.m
This file produces a cloud of _approximate_ possible endpoints for a kinematic
chain given configuration and uncertainty.


## Basic Functions

### robot_arm_endpoints.m
Generally computes the endpoints of a kinematic chain, given the local link
definitions, joint angles, and joint axes.

### link_jacobian.m
Using some information from `robot_arm_endpoints`, this function computes the
jacobian of the specified link, or the instantaneous change in position given
changes in joint angle.

### draw_arm_gaussian.m
Given arm information and a cloud of endpoints, this function plots the
arm, joint axes, and endpoint cloud in 3d space.

### plot_prismatic_arm_distributions.m
Given the distributions of the joint angles and link lengths in a single matrix,
this function produces histograms of the distributions to verify the normal
assumption.