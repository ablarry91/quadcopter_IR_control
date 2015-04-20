##Quadcopter IR Pose Estimation
By, Austin Lawrence
======================

Disclaimer and License
---------------
This project builds on top of the University of Zurich's Matthias Faessler, Elias Mueggler, Karl Schwabe and Davide Scaramuzza's work in estimating the pose of a quadcopter using IR LEDs and correspondence search.  

This package was built and tested using ROS-Indigo, Ubuntu 14.04, an Arduino Uno R3, and a UDI U818A 2.4GHz 4 CH 6 Axis Gyro RC Quadcopter with Camera RTF Mode 2 quadcopter

The source code is released under a GPL licence. Please contact the original authors for a commercial license.


Package Summary
---------------

The RPG Monocular Pose Estimator uses infrared LEDs mounted on a target object and a camera with an infrared-pass filter to estimate the pose of an object.

The positions of the LEDs on the target object are provided by the user in a YAML configuration file. The LEDs are detected in the image, and a pose estimate for the target object is subsequently calculated.


