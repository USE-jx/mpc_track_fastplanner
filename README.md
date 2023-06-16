#  small car

运行效果见B站：https://www.bilibili.com/video/BV1uV4y1m7E8/?spm_id_from=333.999.0.0

## packages介绍

## 1 omni_robot

`功能：` 启动仿真环境，会出现一个带雷达的小车。

`启动：` roslaunch omni_gazebo gazebo.launch 

## 2 lidar2world

`功能：` 把雷达系的点云转到odom系下，传给fastplanner的建图包。

`启动：`  rosrun lidar2world lidar2world_node

## 3 mpc_tracking

`功能：` 跟踪fastplanner生成的局部轨迹

`启动：` rosrun mpc_tracking mpc_tracking_node

`注：`运行这个需要配置非线性优化器，具体可参考我的博客：https://blog.csdn.net/asd22222984565/article/details/130794329

### 4 Fast-planner

`功能：` 实时建立小车前方摄像头可视范围内的障碍物地图，实时地生成避障轨迹，是一个未知环境的局部规划器。

详情请见：https://github.com/HKUST-Aerial-Robotics/Fast-Planner

1. 运行算法：`roslaunch plan_manage 16_lidar.launch`
2. 运行rviz可视化：`roslaunch plan_manage rviz.launch`

