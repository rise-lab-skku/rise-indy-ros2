## Indy bringup
```
ros2 launch indy_bringup indy_bringup.launch.py robot_name:=Nsquare use_fake_hardware:=false
```

## Indy moveit config
moveit_calibration을 사용하기 위해 moveit에 fake_hardware만 올리기 위한 package입니다
```
ros2 launch indy_moveit_config moveit.launch.py use_fake_hardware:=false robot_name:=Indy_RC1 viz:=false
```