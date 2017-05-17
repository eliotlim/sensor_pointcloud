/**
    sonar_pointcloud
    sonar_pointcloud_node.h
    Purpose: ROS Node for creating and managing sonar_pointcloud instances

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/sonar_pointcloud.h>

/**
    Entry point for ROS Node Execution

    @param argc number of arguments
    @param argv argument list
    @return exit status error code, or zero.
*/

int main(int argc, char** argv) {
    // ros::init must be the first call in main()
    ros::init(argc, argv, sonar_pointcloud::ROS_PREFIX);
    ros::NodeHandle nh;

    ROS_INFO("%s: v%s", sonar_pointcloud::ROS_PREFIX.c_str(), sonar_pointcloud::VERSION_STRING.c_str());

    // Remap and resolve Topic Names
    std::string pointcloudTopic = ros::names::append(sonar_pointcloud::ROS_PREFIX, "pointcloud");
    if (ros::names::remap(pointcloudTopic) != pointcloudTopic) {
        pointcloudTopic = ros::names::remap(pointcloudTopic);
    }
    std::string parent_namespace = ros::names::parentNamespace(pointcloudTopic);
    ROS_INFO("Pointcloud Topic mapped to: %s", pointcloudTopic.c_str());

    // TODO: Read Parameters for Topics, Sonar Transforms, etc.

    // TODO: Create SonarPrecipitator Object

    // TODO: Add Sonar Topics to SonarPrecipitator

    // Process all event callbacks
    ros::spin();
}
