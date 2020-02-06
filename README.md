# test_opencv
add linedetect.cpp  

步骤  
```sh
catkin build test_opencv  
rosrun test_opencv linedetect  
```

 更改接收的topic为/image_raw接入usb_cam。    
 目前接收的topic为/camrea/image_raw，用于接收gazebo_models的消息。  

## TODO:  
- [ ] add lineintersection detection  
- [ ] add linecircle detection  
