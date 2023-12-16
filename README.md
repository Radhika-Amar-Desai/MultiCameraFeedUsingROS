# MULTIPLE CAMERA FEED USING ROS

## INTRODUCTION
This project achieves simultaneous live feed from multiple cameras using multithreading. ROS publisher takes live feed from two cameras and publishes it on two seperate topics. A ROS subscriber subscirbes to both topics simultaneously and displays the frames side by side.

## SETUP AND INSTALL

1) Clone the github repo
   `git clone https://github.com/Radhika-Amar-Desai/MultiCameraFeedUsingROS.git`
2) Run the following command in one instance of terminal
   `roscore`  
3) Open another instance of terminal and go to MultiCameraFeedUsingROS folder
   `cd MultiCameraFeedUsingROS/`
4) Now run the following command on the instance of terminal where you opened MultiCameraFeedUsingROS
   `source devel/setup.bash`
5) Now run the ROS publisher in the same instance of terminal as in step 4
   

## DEMONSTRATION

https://github.com/Radhika-Amar-Desai/MultiCameraFeedUsingROS/assets/120047754/c5bf5bb8-d764-4141-85c6-95c95146c53e

## RQT GRAPH

![WhatsApp Image 2023-12-14 at 23 30 54](https://github.com/Radhika-Amar-Desai/MultiCameraFeedUsingROS/assets/120047754/8a591f5e-5838-4901-bb71-e56033e3a156)
