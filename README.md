  <h1 align="center">MWMP-Cpp</h1>
  <h4 align="center">Multi-staged Warm started Motion Planner (MWMP) C++ library</h4>
  
<p align="center">
  <img src="https://user-images.githubusercontent.com/37618448/177987095-dc7dba1f-7879-4f9e-a723-b7c4c3780e14.png" width="200">
</p>

![image](https://user-images.githubusercontent.com/37618448/177952812-e9e866cc-04f3-4659-b53b-97cf3950598f.png)


*Author*: [Gonzalo Jesús Paz Delgado](https://github.com/gonzalopd96), gonzalopd96@uma.es

*Supervisor*: [Carlos J. Pérez del Pulgar](https://github.com/carlibiri), carlosperez@uma.es

*Organization*: [Space Robotics Lab, University of Malaga](https://www.uma.es/space-robotics)

## Table of Contents
  * [Description](#description)
  * [Installation and test scripts](#installation-and-test-scripts)
  * [File tree](#file-tree)
  * [Citation](#citation)
  * [Versions](#versions)


## Description

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) in a Multi-staged Warm-Started manner to plan the movements of a mobile platform. Given the platform kinematic and dynamics model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, Fast Marching Methid (FMM) is used to generate a warm starting trajectory for the mobile base.
  - Second, the unconstrained solution of the motion planning problem is found using Unconstrained SLQ.
  - Third, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete constraints compliant, global motion plan.
  
Check the [Simulation and field tests video](https://youtu.be/xDFv4Ho4KZs).


## Installation and test scripts

The MWMP library includes a set of Unit Tests using [Google Tests](https://github.com/google/googletest). Using the install.sh script the dependencies are scanned and installed, and the library and unit tests are compiled into a build folder. 

```bash
sh install.sh
```

The output should be something similar to:

```bash
Scanning dependencies of target runUnitTests
[  6%] Building CXX object CMakeFiles/runUnitTests.dir/test/unit/MWMPTests.cpp.o
[ 13%] Building CXX object CMakeFiles/runUnitTests.dir/test/unit/StateSpaceModelsTests.cpp.o
[ 20%] Building CXX object CMakeFiles/runUnitTests.dir/test/unit/UnitTests.cpp.o
[ 26%] Building CXX object CMakeFiles/runUnitTests.dir/src/FastMarching.cpp.o
[ 33%] Building CXX object CMakeFiles/runUnitTests.dir/src/MWMP.cpp.o
[ 40%] Building CXX object CMakeFiles/runUnitTests.dir/src/MatrixOperations.cpp.o
[ 46%] Building CXX object CMakeFiles/runUnitTests.dir/src/StateSpaceModels.cpp.o
[ 53%] Building CXX object CMakeFiles/runUnitTests.dir/utils/FileManager/FileManager.cpp.o
[ 60%] Linking CXX executable runUnitTests
[ 60%] Built target runUnitTests
Scanning dependencies of target MWMP
[ 66%] Building CXX object CMakeFiles/MWMP.dir/src/FastMarching.cpp.o
[ 73%] Building CXX object CMakeFiles/MWMP.dir/src/MWMP.cpp.o
[ 80%] Building CXX object CMakeFiles/MWMP.dir/src/MatrixOperations.cpp.o
[ 86%] Building CXX object CMakeFiles/MWMP.dir/src/StateSpaceModels.cpp.o
[ 93%] Building CXX object CMakeFiles/MWMP.dir/utils/FileManager/FileManager.cpp.o
[100%] Linking CXX shared library libMWMP.so
[100%] Built target MWMP
```

Then, to run the unit tests, just use the runUnitTests.sh script:

```bash
sh runUnitTests.sh
```

This should output the result of the unit tests (one unit test for each combination of MWMP stages):

```bash
[==========] Running 7 tests from 2 test suites.
[----------] Global test environment set-up.
[----------] 2 tests from MWMP
[ RUN      ] MWMP.constructors_test
[       OK ] MWMP.constructors_test (44 ms)
[ RUN      ] MWMP.stepped_mp_test
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 0
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.37716 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.185045 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 1
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.384266 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.144376 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 2
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.386507 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.0231699 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 3
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.386917 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.0208145 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 4
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.385364 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.0153954 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 5
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.383254 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.010233 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Iteration number 6
[MotionPlanner::generateUnconstrainedMotionPlan]: Elapsed iteration time: 0.382046 s
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal position: 0.00940097 m
[MotionPlanner::generateUnconstrainedMotionPlan]: Distance to goal orientation: 0.00210437 rad
[MotionPlanner::generateUnconstrainedMotionPlan]: The motion planner found a solution!
[MotionPlanner::generateSteppedMotionPlan]: The imposed constraints are satisfied, the stepped motion planner found a solution!
[MWMP::stepped_mp_test] Elapsed time: 2.81298 s
[       OK ] MWMP.stepped_mp_test (2851 ms)
[----------] 2 tests from MWMP (2895 ms total)

[----------] 5 tests from StateSpaceModels
[ RUN      ] StateSpaceModels.getters_test
[       OK ] StateSpaceModels.getters_test (0 ms)
[ RUN      ] StateSpaceModels.linearized_matrixes_test
[StateSpaceModels::linearized_matrixes_test] Elapsed time matrix A: 1.9e-05 s
[StateSpaceModels::linearized_matrixes_test] Elapsed time matrix B: 5.6e-05 s
[       OK ] StateSpaceModels.linearized_matrixes_test (2 ms)
[ RUN      ] StateSpaceModels.constraints_matrixes_test
[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix C: 5e-06 s
[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix D: 2e-06 s
[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix r: 2e-06 s
[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix G: 4e-06 s
[StateSpaceModels::constraints_matrixes_test] Elapsed time matrix h: 2e-06 s
[       OK ] StateSpaceModels.constraints_matrixes_test (1 ms)
[ RUN      ] StateSpaceModels.costs_matrixes_test
[StateSpaceModels::costs_matrixes_test] Elapsed time matrix Q: 1e-05 s
[StateSpaceModels::costs_matrixes_test] Elapsed time matrix R: 2e-06 s
[StateSpaceModels::costs_matrixes_test] Elapsed time matrix K: 5e-06 s
[       OK ] StateSpaceModels.costs_matrixes_test (2 ms)
[ RUN      ] StateSpaceModels.forward_integrate_test
[StateSpaceModels::forward_integrate_test] Elapsed time forward integration: 6.1e-05 s
[       OK ] StateSpaceModels.forward_integrate_test (0 ms)
[----------] 5 tests from StateSpaceModels (5 ms total)

[----------] Global test environment tear-down
[==========] 7 tests from 2 test suites ran. (2900 ms total)
[  PASSED  ] 7 tests.
Finished execution of runUnitTests.sh
```

Each unit test will save its results in the test/unit/results folder with a particular name (unconstrained, ws_unconstrained, ...).


## File tree
```bash
MWMP-Cpp/
├── src/
│   ├── FastMarching.cpp
│   ├── FastMarching.hpp
│   ├── MatrixOperations.cpp
│   ├── MatrixOperations.hpp
│   ├── MWMP.cpp
│   ├── MWMP.hpp
│   ├── StateSpaceModels.cpp
│   └── StateSpaceModels.hpp
├── test/
│   └── unit/
│       ├── inputs/
│       ├── results/
│       ├── MWMPTests.cpp
│       ├── StateSpaceModelsTests.cpp
│       └── UnitTests.cpp
├── utils/
│   ├── FileManager.cpp
│   └── FileManager.hpp
├── .clang-format
├── .gitignore
├── CMakeLists.txt
├── LICNESE
├── README.md
├── install.sh
└── runUnitTests.sh
```

## Citation

If this work was helpful for your research, please consider citing the following BibTeX entry:

@article{author = {G. J. Paz-Delgado and C. J. Pérez-del-Pulgar and M. Azkarate and F. Kirchner and A. García-Cerezo},
   title = {Multi-stage warm started optimal motion planning for over-actuated mobile platforms},
   url = {http://arxiv.org/abs/2207.14659},
   year = {2022}
}


## Versions

[Go to the MatLab version](https://github.com/spaceuma/MWMP-MatLab)               
[<img src="https://user-images.githubusercontent.com/37618448/177983996-1da1c67d-8037-4b8b-8187-737a8adeee1d.png" width="200">
](https://github.com/spaceuma/MWMP-MatLab)
