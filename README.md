# ROS Connector for DLINK IP cameras
To run this project:
1. Clone this repo into your catkin_ws
2. rosrun RODIPCa connect.py url --name --password --topic --sreen
  * url (String): the url of the camera stream
  * --name (String - Optional): the login name for the camera
  * --password (String - Optional): the login password for the camera
  * --topic (String - Optional): the rostopic to subscribe to
  * --screen (store_true - Optional): shows a gui of the camera stream
