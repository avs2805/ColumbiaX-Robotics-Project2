# ColumbiaX-Robotics-Project2
ColumbiaX-Robotics-Project2 
WIP: Get Interactiver Markers to work on ROS-Melodic


In this project, a robot with a camera mounted on it is modeled and simulated in ROS. To describe the robot poses of all these items, we define various coordinate frames: 'base','robot','camera' and 'object'
We apply the translations (x,y,z) and rotations (quaternions (x,y,z,w)) in a single consolidated form and use tf library from ROS for this purpose.
