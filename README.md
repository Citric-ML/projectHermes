# Project Hermes
A drone navigation algorithm utilizing python for the pathfinding algorithm and C++ for the direct interaction with the drone.

The system is plit into two seperate files, pathfinding and movement calculation. In the hardware I used for this project, the pathfinding algorithm runs on Rasberry Pi and the Movement Calculation file runs on Arduino.

The Pathfinding algorithm is split into 3 main projections; those being the dynamic, static, and heuristic projections. The dynamic projection is hwere points are mapped upon being located in space, and should the drone's comfidence score that the point is static increase to a certain threshold it will transfer that point to the static map, with no decay unlike the dynamic one. The heuristic map is based upon the static one, using four algorithms to perdict a conservative estimate of it's surroundings outside of what has already been observed.

The points are actually mapped onto the grid via a direction vector, using the drones relative x, y, and z position as well as it's direction relative to the starting point to plot a point in a grid projection anchored at the starting point. There are also three bounds limiting how far the drone can explore or expand, varying in properties.
