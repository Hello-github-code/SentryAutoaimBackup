# RMOS_DDS
- 2025 单/双云台哨兵自瞄代码
- 2025.3.18
## 环境配置
    -ros2-humble
    -opencv4.4.0
    -Eigen
## CmakeList.txt
    -需要手动更改每个CmakeList.txt中的3rdparty路径

## 运行节点

- 相机节点： daheng_cam           启动命令：`ros2 run rmos_cam daheng_camera`
- 传统识别节点：basic_detector     启动命令：`ros2 run rmos_detector basic_detector`
- 解算节点：processer             启动命令：`ros2 run rmos_processer processer`
- 通信节点：can_comm              启动命令：`ros2 run rmos_transporter can_comm`

## 如何启动程序
`source install/setup.bash`
- 传统识别
`ros2 launch rmos_bringup normal_aim.launch.py`

## 切换相机
- 使用sn码(serial number)打开指定的相机，若要更换相机，先通过大恒相机驱动，查看sn码及控制模式（control）
  分别修改左右云台rmos_cam文件的sn码启动指定相机


## 需要调试的地方

- 相机的曝光和增益，在rmos_bringup/configure目录下的参数文件中修改

- pnp所用的相机内参，在rmos_bringup/configure目录下的参数文件中修改

- 相机到IMU的位置偏移，在basic_detector.cpp中修改

- 时间戳的同步，可以在can_comm_node.cpp中修改

- 其他所有参数在Algorithm/configure目录下的对应参数文件中修改

## 修改默认优先级
- 优先级决定优先攻击的目标，当同时识别到，正常情况下接收决策发来的优先级信息 AimTarget,若五秒以上未受到决策下发的优先级则会优先级设定为默认优先级 

## debug模式

- 在Algorithm/configure/Debug目录下的参数文件中，有一个debug.xml，里面展示了支持的debug选项，并且支持在程序运行过程中实时修改这些选项，如果打开第一个选项contest，则进入比赛模式，将关闭所有debug信息的显示。
- 其他debug信息通过ros2 topic echo的方式查看。
- 图像信息通过rqt查看，包括原始图像，框出装甲板后的图像，二值化图像。


## 注意事项

### 如果launch不起来

- 拔插相机线
- 看是否有IMU数据，拔插
- 整车重启
- 每个节点分开run
- 如果出现file too short错误，请删除build，log，install三个文件夹之后重新编译
- 看是否有僵尸话题，参考文章：https://answers.ros.org/question/377224/how-do-you-kill-the-zombie-node-in-ros2-on-local/


## TODO

- 分类数据集中没有5号平衡
- 和大符代码合并



## 其他

代码与之前有许多改动，并且测试可能不足，有问题及时反馈



#### 

#### 
