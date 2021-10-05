# SOMP-Cpp
Stepped Optimal Motion Planning (SOMP) C++ library

Space Robotics Lab, University of Malaga
Author: Gonzalo Jesús Paz Delgado, gonzalopd96@uma.es
Supervisor: Carlos J. Pérez del Pulgar, carlosperez@uma.es

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) and a Constrained SLQ algorithms to plan the movements of a robot. Given the robot kinematic and dynamic model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, the unconstrained solution of the motion planning problem is found using SLQ.
  - Second, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete non-linear, constrained, global motion plan.
