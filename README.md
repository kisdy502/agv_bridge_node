# 功能简介
提供WebSocket server端，接收仿真agv发过来的控制指令，将agv硬件层的位置，姿态等数据发送给仿真机器人
调用仿真机器人提供的agv 控制接口

# 编译  agv_bridge_v2  （agv_bridge 是python版本的，没有开发好，不用了，就用C++版本）
colcon build --packages-select agv_bridge_v2  --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch
```
source install/setup.bash
ros2 launch agv_bridge_v2 agv_bridge_v2.launch.py use_sim_time:=True
```

# 方法2: 直接运行节点
```
source install/setup.bash
ros2 run agv_bridge_v2 agv_bridge_node \
  --ros-args \
  -p use_sim_time:=True \
  -p agv_id:=AGV001 \
  -p springboot_host:=192.168.3.4 \
  -p springboot_port:=22777 \
  -p websocket_port:=9090
```
# python 环境冲突解决  退出当前 conda 环境

```
conda deactivate
```


# ros2-control 安装
sudo apt install ros-$ROS_DISTRO-ros2-control
ros2 control list_controller_types
ros2 control list_controllers
sudo apt info ros-$ROS_DISTRO-ros2-controllers
sudo apt install  ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-gazebo-ros2-control