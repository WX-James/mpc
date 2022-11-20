# 基于线性模型预测控制的差速小车轨迹跟踪控制器

本仓库来源于浙江大学Fast-lab实验室的3D2M_planner项目的mpc轨迹跟踪器，笔者根据项目需求将其提取和修改作为独立的功能包，并提供使用说明。
(https://github.com/ZJU-FAST-Lab/3D2M-planner)

## 1. 依赖安装

（1）OSQP安装

```bash
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make
sudo make install
```

（2）OSQP-Eigen安装

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build 
cd build
cmake ..
make
sudo make install
```

## 2. 编译使用

（1）下载编译

```bash
cd catkin_ws/src
git clone https://github.com/WX-James/mpc
cd .. && cakin_make
```

（2）文件结构

```
├── CMakeLists.txt
├── config
│   ├── param copy.yaml
│   └── param.yaml
├── include
│   ├── cubic_spline_planner.h
│   ├── mpc.h
│   └── mpc_utils
│       ├── minco.hpp
│       ├── poly_traj_utils.hpp
│       ├── root_finder.hpp
│       └── traj_anal.hpp
├── launch
│   └── test_mpc.launch
├── msg
│   └── Polynome.msg
├── package.xml
└── src
    ├── cubic_spline_planner.cpp
    ├── mpc.cpp
    └── mpc_node.cpp
```

（3）启动文件参数配置

​		launch文件：/mpc/launch/test_mpc.launch

| 参数                  | 说明                                                         |
| --------------------- | ------------------------------------------------------------ |
| arg name="odom_topic" | 机器人里程Topic，用于MPC获取机器人当前位置                   |
| arg name="traj_topic" | 参考轨迹Topic，消息类型是自定义类型，msg文件在/mpc/msg/Polynome.msg |
| arg name="cmd_topic"  | 差速小车的控制指令，cmd_vel：线速度和角速度                  |

（4）MPC算法参数配置

​		在yaml文件：/mpc/config/param.yaml中对MPC轨迹跟踪器进行具体参数配置

（5）发布参考轨迹的说明

​		请按照自定义消息/mpc/msg/Polynome.msg来发布参考轨迹

（6）启动MPC轨迹跟踪控制器

```bash
source devel/setup.bash
roslaunch mpc test_mpc.launch
```

