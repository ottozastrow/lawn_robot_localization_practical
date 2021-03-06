# Beginner Practical: Lawn Robot Localization
The goal of this project was to learn about ROS, Sensors and Kalman filters, as well as to employ filtering of GPS and IMU data, in order to test whether filtering increases the localization robustness.

## Project Overview
The filtering was performed using an [Extended Kalman Filter](https://www.spiedigitallibrary.org/conference-proceedings-of-spie/3068/1/New-extension-of-the-Kalman-filter-to-nonlinear-systems/10.1117/12.280797.short?SSO=1) (EKF) implementation from the [robot localization package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html). In our case the Kalman Filter recieves data from two GPS sensors and an IMU sensor.

This repository includes several nodes to enable publishing and processing sensory data and the respective tf transforms. We also provide scripts for data visualization and a configuration file for the robot_localization package.
The measurement data is available in the form of bagfiles.


### links
link to bagfiles: https://drive.google.com/file/d/16mKE89SpYCWMkU_XrrvHeFVmerjwThBf/view?usp=sharing
link to Google Slides presentation: https://docs.google.com/presentation/d/1UXo_ObpeUY94QeIahlgv_8dERTovL3SQyROQ8bVYJHY/edit?usp=sharing



## Usage

``roslaunch robotics_practical playback.launch bagfile:=<BAG_FILE>``
replace <BAG_FILE> with the filepath to your bagfile. For example: ``"./bagfiles/with_vectornav_output/filtered_topics/2019-10-25-12-36-48.bag"``

The launchfile will replay the chosen bagfile, launch a preconfigured rviz session, perform all necessary transforms, start the EKF node, log all the odometry data and save it to csv files for visualization

The output data is stored in ``output_data/`` in one csv file per Odometry publisher. This data can be visualized with by using ``python3 plot_odom.py --input=<DATA_DIR>`` and providing a path to the folder containing the csv files. Optionally the ``--no_gt``can be set signal that the ground truth data should not be plotted.

## Experiments
The plot in the figure below can be obtained from the plot_odom.py script as outlined above. It depicts selected Odometry publishers from a birds eye view. 

![Experiment 1](images/experiment1.png)
