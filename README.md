### Overview
This repository holds code of a [ROS](http://www.ros.org) package for point cloud acquisition with a [Gocator3109A](http://lmi3d.com/products/gocator/snapshot-sensor) camera. It is basically a ROS wrapper of the low-level API provided by [LMI Technologies](http://lmi3d.com), the manufacturer of the camera. 

![Camera and cloud at rviz](https://github.com/beta-robots/gocator_3100/blob/master/media/gocator_3100_ros_pkg.png)

### Dependencies
The package has been tested with the following dependencies:
* Ubuntu 14.04
* CMake + gcc
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [Point Cloud Library v1.7](http://www.pointclouds.org/) (shipped with ROS Indigo)
* GoSDK library (propietary library from manufacturer LMI Technologies)

To install GoSDK dependency, the following steps are required: 

1. With a Serial Number of a device, register as a customer at the [LMI website](http://downloads.lmi3d.com/)
2. Download the SDK from the [LMI website](http://downloads.lmi3d.com/) (file 14400-4.2.5.17_SOFTWARE_GO_SDK.zip)
3. Uncompress GoSDK
4. Build GoSDK
```shell 
$ cd GO_SDK/Gocator
$ make -f GoSdk-Gnu.mk 
```

### Download
```shell
$ git clone https://github.com/beta-robots/gocator_3100.git
```

### Build
1. Indicate, by editing the CMakeLists.txt of this package (line 58), where GoSDK library is placed (This point should be improved , see issue #7)
2. The build procedure is the ROS-catkin standard.From your ROS workspace: 
```shell
$ catkin_make --only-pkg-with-deps gocator_3100 
```

### Execute

1. Plug your camera correctly, in terms of power, signal and **grounding!**
2. Properly edit the config/gocator_3100_params.yaml according your configuration and needs. 
3. Call the launch file. From the package folder would be
```shell
roslaunch launch/gocator_3100.launch
```



