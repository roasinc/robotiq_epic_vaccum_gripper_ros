# robotiq_epic_vaccum_gripper_ros
ROS packages for control RobotiQ EPick Vaccum Gripper


## View Gripper
```
$ roslaunch robotiq_epick_vaccum_gripper_description view_gripper.launch
```


## Run Gripper
```
$ rosrun robotiq_epick_vaccum_gripper_control robotiq_epick_vaccum_gripper_control_node
```

### Parameters
```
    port_name:=/dev/ttyUSB0
    baudrate:=115200
    rate:=20.0
    gripper_timeout:=0.0
```


## Topic
```
/gripper_command [control_msgs/GripperCommand]

float64 position [1.0: grip, 0.0: release]
float64 max_effort [not_used]
```

```
/gripper_status [robotiq_epick_vaccum_gripper_msgs/GripperStatus]

uint8 g_act [gripper_activated]
uint8 g_mod [current mode: 0 - automatic, 1 - advanced]
uint8 g_gto [regulate: 0 - stop, 1 - follow command]
uint8 g_sta [activation status: 0 - not activated, 3 - activated]
uint8 g_obj [object detection: 0 - unknown, 1 - object detected, minimum vaccum, 2 - object detected, maximum vaccum, 3, no object]
uint8 g_vas [vaccum actuator: 0 - standby, 1 - vaccum on, 2 - passive release, 3 - active release]
uint8 g_flt [gripper fault]
uint8 k_flt [?]
uint8 g_pr [request vaccum echo]
uint8 g_po [actual vaccum]
```