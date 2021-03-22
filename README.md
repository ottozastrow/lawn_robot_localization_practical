# lawn_robot_localization_practical
The goal of this project was to learn about ROS, Sensors and Kalman filters, as well as to employ filtering of GPS and IMU data, in order to test whether filtering increases the localization robustness.

## Approach
The filtering was performed using an Extended Kalman Filter [-@Simon J. Julier and Jeffrey K. Uhlmann] (EKF) implementation from the robot localization package. In our case the Kalman Filter recieves data from two GPS sensors and an IMU sensor.

This repository includes several nodes to enable publishing and processing sensory data and the respective tf transforms. We also provide scripts for data visualization and a configureation file for the robot_localization package.
The measurement data is available in the form of bagfiles


### links
link to bagfiles: https://drive.google.com/file/d/16mKE89SpYCWMkU_XrrvHeFVmerjwThBf/view?usp=sharing
link to google slides presentation: https://docs.google.com/presentation/d/1UXo_ObpeUY94QeIahlgv_8dERTovL3SQyROQ8bVYJHY/edit?usp=sharing

## Experiments

## Usage


---
references:
author : Simon J. Julier and Jeffrey K. Uhlmann
title : New extension of the Kalman filter to nonlinear systems
volume : 3068
booktitle : Signal Processing Sensor Fusion and Target Recognition VI
editor : Ivan Kadar
organization : International Society for Optics and Photonics
publisher : SPIE
pages : 182 -- 193
year : 1997
doi : 10.1117/12.280797
URL : https://doi.org/10.1117/12.280797
---
