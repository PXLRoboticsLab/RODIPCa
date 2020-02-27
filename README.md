# ROS Connector for DLINK IP Cameras
A simple ROS python script to connect DLINK IP cameras with ROS.

## Getting started
### Prerequisites
For this project to work you need to install the following dependencies:
* OpenCV 3
* ROS

### Install
Follow the steps below to install and run the software on your local machine:
1. Clone this repo into your catkin_ws
2. rosrun RODIPCa connect.py url [--name] [--password] [--topic] [--screen]
   * `url` (String): the url of the camera stream (make sure the url is a mjpeg stream)
   * `--name` (String - Optional): the login name for the camera
   * `--password` (String - Optional): the login password for the camera
   * `--stream` (String - Optional): the video stream to be used (default: video1.mjpg)
   * `--topic` (String - Optional): the name for the rostopic the data will be published to (default: dlink/image/compressed)
   * `--fps` (int - optional): the maximum framerate of the camera (default: 30)
   * `--screen` (store_true - Optional): shows a gui of the camera stream

## License
This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/PXLRoboticsLab/RODIPCa/blob/master/LICENSE.md)  file for details.

## DLINK cameras that work with this project
* [DLink DCS-4602EV](http://www.dlink.com/uk/en/products/dcs-4602ev-full-hd-outdoor-vandal-proof-poe-dome-camera)
* [DLink DCS-4802E](http://us.dlink.com/products/business-ip-cameras/vigilance-full-hd-outdoor-mini-dome-network-camera/)

**If you tested the project with another IP camera that's not in the list send an email to: Tim.Dupont@pxl.be so we can add it to the list.**

## Authors
* [Maarten Bloemen](https://github.com/MaartenBloemen) 
