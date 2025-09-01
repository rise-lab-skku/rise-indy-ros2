moveit_calibration을 사용하기 위해 moveit에 fake_hardware만 올리기 위한 package입니다
- launch
    ```
    ros2 launch indy_bringup indy_bringup.launch.py robot_name:=Nsquare use_fake_hardware:=false
    ```