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
    ROS_INFO("Pointcloud Topic : %s", pointcloudTopic.c_str());

    // \Read Parameters for Topics, Sonar Transforms, etc.
    std::string pointcloudFrame;
    nh.param<std::string>(ros::names::append(parent_namespace, "pointcloudFrame"), pointcloudFrame, "map");
    ROS_INFO("Pointcloud Frame : %s", pointcloudFrame.c_str());

    // Create SonarPrecipitator Object
    SonarPrecipitator precipitator(pointcloudTopic, pointcloudFrame);

    ROS_INFO("SonarPrecipitator created");

    // Add Sonar Topics to SonarPrecipitator
    std::vector<std::string> sonars;
    nh.getParam("sonars", sonars);
    if (sonars.size() == 0) ROS_WARN("No Sonars configured");

    for (std::vector<std::string>::iterator sonarNameIt = sonars.begin(); sonarNameIt != sonars.end(); ++sonarNameIt) {
        std::string sonarTopic, sonarFrame;
        nh.getParam(*sonarNameIt + "/topic", sonarTopic);
        nh.getParam(*sonarNameIt + "/transform/frame", sonarFrame);
        ROS_INFO("Sonar Parameters Loaded - Topic: %s Frame: %s", sonarTopic.c_str(), sonarFrame.c_str());

        boost::shared_ptr<Sonar> s = precipitator.addSonar(sonarTopic, sonarFrame);

        // Determine if transform is defined in yaml
        bool loadTransform = false;
        double translation[3] = {0, 0, 0};
        // Load x/y/z parameters
        for (char c = 'X'; c <= 'Z'; c++) {
            std::string paramStr = *sonarNameIt + "/transform/pos" + c;
            if (nh.getParam(paramStr, translation[c - 'X'])) {
                loadTransform = true;
                ROS_INFO("Loading Transform for %s: %f", paramStr.c_str(), translation[c-'X']);
            }
        }
        if (loadTransform) {
            // Load roll/pitch/yaw parameters - WILL NOT LOAD IF x/y/z not set
            tf2::Quaternion q;
            float roll = 0, pitch = 0, yaw = 0;
            nh.getParam(*sonarNameIt + "/transform/roll", roll);
            nh.getParam(*sonarNameIt + "/transform/pitch", pitch);
            nh.getParam(*sonarNameIt + "/transform/yaw", yaw);
            q.setRPY(roll, pitch, yaw);

            // Set sensor transform from parameters
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.frame_id = pointcloudFrame;
            transformStamped.transform.translation.x = translation[0];
            transformStamped.transform.translation.y = translation[1];
            transformStamped.transform.translation.z = translation[2];
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            s->setTransform(transformStamped);
            ROS_INFO("Sonar Transform Loaded - Frame: %s (%.2f, %.2f, %.2f) rot: {%.2f, %.2f, %.2f}",
                     sonarFrame.c_str(), translation[0], translation[1], translation[2],
                     roll, pitch, yaw);
        }
    }

    ROS_INFO("Sonars added");

    // Process all event callbacks
    ros::spin();
}
