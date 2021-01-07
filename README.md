# aubo_pick_up_with_6D
## 1. aubo_i5驱动安装

1. 源码下载：
   ``git clone https://github.com/lg609/aubo_robot.git``
2. 依赖安装：
   ``sudo apt-get install ros-kinect-moveit-**``
3. if 编译报错
   ``.so.0.15``
   解决方法：
   	由 /opt/ros/lib/ 复制出提示包，改为对应版本后重新复制至该目录下。
	&&手动复制 UpdateMoveitLib/Kinetic/下的.so文件至/opt/ros/lib/中
	or
	重新配置moveit，修改launch文件中controller文件&&moveit_planning_execution&&config文件夹下的controller.yaml

## 2. dope 安装

修改config下的pose.yaml文件进行修正，如doc文件下的config_pose.yaml所示

1. cv2 & ros 冲突错误解决
	**1.1**  安装ros-opencv
   ``sudo apt-get install ros-kinect-version-opencv``
	&
	安装opencv opencv-contrib-python
	``python -m pip install opencv-python``
	``python -m pip install opencv-contrib-python``
	
	**1.2** 修改引用路径
	``import sys``
	``sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')``
	``import cv2``
	``sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')``
	
	**1.3** 修改python文件顶部环境
	``#!/usr/bin/env python3 -->  #!/usr/bin/env python2``
2. 显卡驱动安装
	1050Ti对应 380驱动，使用ppa安装，**关掉boot的安全模式**
3. 权重
	如文件夹weights所示
	
## 3. 手眼标定

**3.1** 安装realsense
	直接由源码安装即可，需要安装所有依赖项

```
 echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
   sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
   sudo apt-get update``
   sudo apt-get install librealsense2-dkms
   sudo apt-get install librealsense2-utils
   sudo apt-get install librealsense2-dev
   sudo apt-get install librealsense2-dbg
```

**3.2** 安装arcuo_ros

``git clone -b kinetic-devel https://github.com/pal-robotics/aruco_ros``
``catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="aruco_ros"``

安装后注意先调试arcuo_ros，打开realsense，启动arcuo_ros 在rviz下查看arcuo_ros->result->image 看是否成功；启动launch详见doc文件夹下的*marcuo.launch*
*Notes:* 
   - Dictionary 一定要选 Original ArUco
   - Marker ID 和 Marker size 自选，在launch 文件中做相应的修改, launch文件中默认单位为m
   - 打印时，要选择原始大小，否则要测量一下打印出来的真实大小

**3.3** vision_visp安装
   ``sudo apt-get install ros-kinetic-visp``

**3.4** easy_handeye安装
``git clone https://github.com/IFL-CAMP/easy_handeye``
``catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="easy_handeye"``
*Notes:*

	- ``pip install transforms3d``
	- ``python -m pip install opencv-contrib-python``
	- 在cv2.CALIB_HAND_EYE_TSAI出错的py文件中修改如下语句
		``import sys
	    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	    import cv2
	    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')``
	- 未解决 gui无法打开报错

**3.5** 标定流程

1. 依次启动相应launch 详见doc下
    - mrealsense.launch
    - maruco.launch
    - maubo.launch
    - measy_hand.launch

*Notes:* 

   - 在 *maruco.launch* 中 *reference_frame* & *camera_frame* 应填入光轴 *camera_color_optical_frame*；即标定应该是工具坐标系 & 光轴
        - 在 *measy_hand.launch* 中 *tracking_base_frame* 填 *camera_color_optical_frame*；*robot_base_frame* 填入arm的基坐标 *base_link*；
        *robot_effector_frame* 填入工具坐标 *wrist3_Link*
   - camera在手上，则*eye_on_hand = true*，在手外则 *eye_on_hand = false* 	

2. 在easy_handeye的rqt界面下
	- check starting pose
	- 点击next pose，再点击plan，出现绿色就当前位姿可以达到，最后点击execute。出现红色表示当前点不可用，继续点击next pose，重复操作；
	- 查看image_view中是否跟踪到标签，如果跟踪到标签则点击take sample，如果没有跟踪到则跳过。重复（1）（2）操作，直到17个点全部采集完；
	- 点击compute
	- 点击save，保存得到的结果。结果保存在*～/.ros/easy_handeye*的.yaml文件中。本次标定结果如*doc/auboi5_realsense_handeyecalibration_eye_on_hand.yaml*所示

3. 修改 & 运行 easy_handeye下的publish.launch
	- *eye_on_hand* 参数设置如上所示
	- *namespace* 和 *measy_hand.launch* 中的 *namespace* **相同**
	- 运行 ``roslaunch publish.launch``

## 4. 最终控制
依次运行
- mrealsense.launch
- dope.launch
- maubo.launch
- rosrun control tf_camera
- python Robotiq2FGripperController.py
- rosrun control pick5

*Notes:* 运行python文件时应将所有py文件 chmod 777


