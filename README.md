### 开源SLAM算法：VINS-MONO 跑 RealSense-ZR300 教程.
    一、安装ROS系统
    安装教程详见ROS wiki网址: http://wiki.ros.org/cn/kinetic/Installation/Ubuntu

    二、安装RealSense-ZR300驱动
    安装教程详见intel官网：注意安装完必须存在 librealsense.so 库文件.

    三、安装realsense_camera驱动包
    安装教程详见ROS wiki网址：http://wiki.ros.org/realsense_camera

    四、标定RealSense-ZR300传感器

    五、用ZR300实际运行VINS-MONO算法
    1. 先跑ZR300驱动程序：
    roslaunch realsense_camera zr300_nodelet_default.launch

    2. 再下载并编译运行VINS-MONO程序：
    mkdir catkin_ws_vin/src/
    cd catkin_ws_vin/src/
    git clone https
    cd ..
    catkin_make
    source devel/setup.bash
    roslaunch vins_estimator realsense_color.launch
	
    3.最后运行可视化节点RVIZ
    cd catkin_ws_vins
    source devel/setup.bash
    roslaunch vins_estimator vins_rviz.launch










	
	
	
	
