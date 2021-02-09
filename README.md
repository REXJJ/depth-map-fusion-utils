# MovingCameraFusion
A repository for fusing pointcloud data using a moving camera.

The include folder contains the implementations of the various algorithms discussed and the tests folder contains the unit tests for each includes.

The files in the include folder: 
i) Algorithms.hpp

Contains methods for:
- Set Cover
- Sampling a sphere
- Optimizing camera based on the optimal distance. 

ii) Camera.hpp
Contains a camera class and various methods for working with the depth cameras. Used in ray tracing.

iii) RayTracingEngine.hpp
Contains various methods for ray tracing and getting simulated clouds for a rgb-d camera.

iv) TSPSolver.hpp
Solves the TSP problem using 2-opt method.

v) Volume.hpp
THis contains an occupancy grid data structure for ray tracing mechanisms.

vi) VisualizationUtilities.hpp
Contains various visualization utilities for visualizing the pointcloud and the occupancy grid data structure and cameras. THis also has a class for allowing asychronous visualization of pointclouds.


