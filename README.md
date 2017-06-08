# ROS Connector for DLINK IP Cameras
## To run this project:
1. Clone this repo into your catkin_ws
2. rosrun RODIPCa connect.py url --name --password --topic --screen
   * url (String): the url of the camera stream
   * --name (String - Optional): the login name for the camera
   * --password (String - Optional): the login password for the camera
   * --topic (String - Optional): the rostopic to subscribe to
   * --screen (store_true - Optional): shows a gui of the camera stream

## DLINK cameras that work with this project
* [DLink DCS-4602EV](http://www.dlink.com/uk/en/products/dcs-4602ev-full-hd-outdoor-vandal-proof-poe-dome-camera)

**If you tested the project with another IP camera thats not in the list send an email to: maarten.bloemen@student.pxl.be so we can add it to the list.**
