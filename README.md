# lawn_robot_localization_practical
The goal of this project was to learn about ROS, Sensors and Kalman filters, as well as to employ filtering of GPS and IMU data, in order to test whether filtering increases the localization robustness.

## Approach
The filtering was performed using an [Extended Kalman Filter](https://www.spiedigitallibrary.org/conference-proceedings-of-spie/3068/1/New-extension-of-the-Kalman-filter-to-nonlinear-systems/10.1117/12.280797.short?SSO=1) (EKF) implementation from the [robot localization package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html). In our case the Kalman Filter recieves data from two GPS sensors and an IMU sensor.

This repository includes several nodes to enable publishing and processing sensory data and the respective tf transforms. We also provide scripts for data visualization and a configureation file for the robot_localization package.
The measurement data is available in the form of bagfiles


### links
link to bagfiles: https://drive.google.com/file/d/16mKE89SpYCWMkU_XrrvHeFVmerjwThBf/view?usp=sharing
link to google slides presentation: https://docs.google.com/presentation/d/1UXo_ObpeUY94QeIahlgv_8dERTovL3SQyROQ8bVYJHY/edit?usp=sharing

## Experiments

## Usage

``roslaunch robotics_practical playback.launch bagfile:="/bagfiles/with_vectornav_output/filtered_topics/2019-10-25-12-36-48.bag"``
