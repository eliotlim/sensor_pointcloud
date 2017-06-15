# sonar_pointcloud
Convert range data from sonar/lidar topics (`sensor_msgs::Range`) to pointclouds (`sensor_msgs::PointCloud2`)

## Installation
### Using catkin_make
1. Change directory to your catkin workspace
`cd ~/catkin_ws`
2. Clone the repository to your catkin workspace
`git clone https://github.com/eliotlim/sonar_pointcloud`
3. Build the package from source by performing `catkin_make`
---
## Usage
### roslaunch
1. Execute `roslaunch sonar_pointcloud sonar_pointcloud.launch`
  - This will load `sonar_pointcloud/sonarConfig.yaml` by default.

### rosrun
1. Load ros parameters using `rosparam`
2. Call `rosrun sonar_pointcloud sonar_pointcloud_node`

### How to use sonarConfig.yml
Please refer to [this guide](sonarConfig.md) for details.

---
## Results
Launching the node using roslaunch with the default `sonarConfig.yml`
will give you the following transform tree.
![img](extras/side.png) ![img](extras/top.png)
---
## Miscellaneous
---
### License
This library is licensed under the MIT License.
