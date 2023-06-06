# YOLO node for ICUAS

1. make sure that OpenCV version >= 4.4 so that darknet could be deployed.
2. clone this repo under some random workspace, e.g., lala_ws/src
3. catkin_make
4. modify yolo.launch under launch folder accordingly to set the path of cfg and weight files
5. launch yolo.launch
6. open rqt_image_view and check topic /processed_image