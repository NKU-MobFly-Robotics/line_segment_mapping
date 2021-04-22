# 2-D Line Segment Mapping

Source code of an incremental and consistent 2-D line segment-based mapping approach proposed in 

> J. Wen, X. Zhang, H. Gao, J. Yuan, and Y. Fang, "CAE-RLSM: Consistent and efficient redundant line segment merging for online feature map building,", ***IEEE Transactions on Instrumentation and Measurement***, 2020, 69(7): 4222-4237. [[paper]](https://ieeexplore.ieee.org/document/8882497) [[video]](https://youtu.be/025_dzmVGWY) 

The 2-D line segment feature is extarcted by a seeded region growing-based line segment extarction algorithm proposed in

> H. Gao, X. Zhang, Y. Fang, and J. Yuan, "A line segment extraction algorithm using laser data based on seeded region growing," ***International Journal of Advanced Robotic Systems***, 2018, 15(1): 1-10. [[paper]](https://journals.sagepub.com/doi/full/10.1177/1729881418755245)[[code]](https://github.com/NKU-MobFly-Robotics/laser-line-segment)

Please cite the above papers if you use this repo in your research.

## How to use

This repo has been tested on Ubuntu 16.04 and 18.04. The line segment mapping module is integrated into a 2-D pose graph SLAM system, which uses open karto package as the front-end and g2o solver as the back-end. Please check out [this repo](https://github.com/nkuwenjian/slam_karto_g2o) to install g2o solver.

After installing g2o solver, please create and initialize a ROS workspace. We assume that your workspace is named catkin_ws. Then, run the following commands to clone and build open karto package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-perception/open_karto.git
$ cd ..
$ catkin_make
```

After the above preparation, clone and build this package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/line_segment_mapping.git
$ cd ..
$ catkin_make
```

Finally, run the following commands to launch Karto SLAM with line segment mapping:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch line_segment_mapping line_segment_mapping.launch
```

Open a new terminal and play your rosbag:
```
$ rosbag play <rosbagfile> --clock
```