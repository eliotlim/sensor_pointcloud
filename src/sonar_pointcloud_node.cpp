/**
    sonar_pointcloud
    sonar_pointcloud_node.h
    Purpose: ROS Node for creating and managing sonar_pointcloud instances

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/sonar_pointcloud.h>
#include <sonar_pointcloud/SonarPrecipitator.h>

using namespace sonar_pointcloud;

/**
    Entry point for ROS Node Execution

    @param argc number of arguments
    @param argv argument list
    @return exit status error code, or zero.
*/

int main(int argc, char** argv) {
    // ros::init must be the first call in main()
    ros::init(argc, argv, ROS_PREFIX);
    ros::NodeHandle nh;

    ROS_INFO("%s: v%s", ROS_PREFIX.c_str(), VERSION_STRING.c_str());

    // Remap and resolve Topic Names
    std::string pointcloudTopic = ros::names::append(ROS_PREFIX, "pointcloud");
    if (ros::names::remap(pointcloudTopic) != pointcloudTopic) {
        pointcloudTopic = ros::names::remap(pointcloudTopic);
    }
    std::string parent_namespace = ros::names::parentNamespace(pointcloudTopic);
    ROS_INFO("Pointcloud Topic mapped to: %s", pointcloudTopic.c_str());

    // TODO: Read Parameters for Topics, Sonar Transforms, etc.
    std::string pointcloudFrame = "base";

    // Create SonarPrecipitator Object
    SonarPrecipitator precipitator(pointcloudTopic, pointcloudFrame);

    ROS_INFO("SonarPrecipitator created");

    // TODO: Add Sonar Topics to SonarPrecipitator
    std::string sonarTopic = "/ultrasound/range", sonarFrame = "ultrasound";
    boost::shared_ptr<Sonar> s = precipitator.addSonar(sonarTopic, sonarFrame);

    // TODO: Read and set sensor transform from parameters
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = pointcloudFrame;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.8;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    s->setTransform(transformStamped);

    ROS_INFO("Sonars added");

    // Process all event callbacks
    ros::spin();
}
