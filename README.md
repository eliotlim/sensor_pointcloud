# sonar_pointcloud
Convert range data from sonar/lidar topics (`sensor_msgs::Range`) to pointclouds (`sensor_msgs::PointCloud2`)

## Installation
### Using catkin_make
1. Change directory to your catkin workspace
`cd ~/catkin_ws`
2. Clone the repository to your catkin workspace
`git clone https://github.com/eliotlim/sonar_pointcloud`
3. Build the package from source by performing `catkin_make`

## Usage
### Using roslaunch
1. `roslaunch sonar_pointcloud sonar_pointcloud.launch`
2. This will load `sonar_pointcloud/sonarConfig.yaml` by default.

### Using rosrun
1. Load ros parameters using `rosparam`
2. Call `rosrun sonar_pointcloud sonar_pointcloud_node`

### Using sonarConfig.yml

## Miscellaneous

### License
This library is licensed under the MIT License.
