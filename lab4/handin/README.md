## Implementation of lab 4
## GROUP 1
## Harald Lilja
## Anton Olsson
## Simon Brunaer

### Motion detection
$ python pi/basic-motion-detection/motion_detector.py

### Matlab brake light simulator
$ javac -lcm cp.jar lcm/exlcm/*.java
$ jar cf lcm/my_types.jar lcm/exlcm/*.class
## Put lcm.jar in lcm/ directory
## In matlab run listener.m


### Robot controller
## when running the controller, the id corresponds to the position of the robot
$ python lcm/controller.py

### Robot controller simulation
$ python lcm/controller-sim.py
