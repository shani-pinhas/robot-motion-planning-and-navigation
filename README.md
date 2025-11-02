This project focuses on robot motion planning and navigation using a polygonal robot model.
In the first part, a MATLAB implementation generates the configuration space (C-space) by computing all possible robot–obstacle interactions over 32 rotation layers. The space is discretized into a 3D binary grid (32×32×32), where each cell represents the robot’s valid or forbidden position for a given orientation. This forms the geometric foundation for path planning.

In the second part, the A* algorithm is applied to find the shortest path in the discretized configuration space. Both offline (complete map known) and online (sensor-based exploration) navigation scenarios were simulated. The online version dynamically updates the robot’s map using obstacle detection and replans in real time when new obstacles appear.

Results demonstrate successful navigation from start to goal positions, efficient obstacle avoidance, and clear visualization of the A* search ellipses that form around the start–goal pair in each θ-layer.
