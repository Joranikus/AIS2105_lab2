# AIS2105_lab2

# 🚀 Launching with Launch File

## Terminal 1
```bash
source ~/lab2_ws/install/setup.bash
ros2 launch pid_controller pid_controller_launch.py kp:=0.05 ki:=0.1 kd:=0.0
```

## Terminal 2 (Service)
```bash
source ~/lab2_ws/install/setup.bash
ros2 run pid_controller reference_input_node
```




---

# 🛠 Running Nodes Manually (Without Launch File)

## Terminal 1 – Start Joint Simulator
```bash
source ~/lab2_ws/install/setup.bash
ros2 run joint_simulator joint_simulator_node
```

## Terminal 2 – Start PID Controller Node
```bash
source ~/lab2_ws/install/setup.bash
ros2 run pid_controller pid_controller_node --ros-args -p p:=0.05 -p i:=0.1 -p d:=0.0
```

## Terminal 3 – Start Reference Input Node
```bash
source ~/lab2_ws/install/setup.bash
ros2 run pid_controller reference_input_node
```
