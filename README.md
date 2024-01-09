# Offline-SLAM

SLAM - Simultanous Localization and Mapping

## Table of contents
* [General information](#general-information)
* [Requirements](#requirements)
* [System preparation](#requirements)
* [Measurements](#measurements)
* [GUI](#gui)
* [Pose Estimation and Map Alignment](#pose-estimation-and-map-alignment)
* [Connection to ROS](#connection-to-ros)

## General information

The goal of the project is to provide a functionality of point cloud alignment of maps that are measured in different locations in a room. Project aims to provide pose estimation for each measurement. The solution can be implemented for a real LiDAR data from a robot.

## Requirements

* Python version (Windows): Python 3.9.1
* Virtual Machine: Ubuntu 16.04.7 LTS Xenial
* ROS version (VM): kinetic

## System preparation

LiDAR:
![ScreenShot](/screenshots/LiDAR.png)
Robot:
![ScreenShot](/screenshots/robot.png)
Measurements:
![](/screenshots/robot-driving.gif)

## Measurements

To operate Jaguar robot, the following ROS package was installed and used:
https://github.com/gitdrrobot/jaguar4x4_2014
Static and Dynamic Measurements were done. 
Static:
![ScreenShot](/screenshots/static-scan.png)
Measurements on a robot:
![ScreenShot](/screenshots/dynamic-scan.png)

## GUI

GUI was created with the use of PyQt library. It has widgets to link the point cloud input and output directories. Additionally, 3D and 2D view is available. The application allows to start the calculations that include rotation and translation. 
![ScreenShot](/screenshots/gui.png)

## Pose Estimation and Map Alignment

After calculations are done, pose estimation can be seen at the Localization panel:
![ScreenShot](/screenshots/pose.png)
Merged maps:
![ScreenShot](/screenshots/map.png)

## Connection to ROS

ROS node for checking connection to LiDAR was implemented. Listeners are initialized in Python application and they retrive ROS messages on connection status. 
![ScreenShot](/screenshots/rosbridge.png)

The result is seen in GUI. It allows to detect the connection failure in real-time.  
![ScreenShot](/screenshots/gui-with-ros.png)
