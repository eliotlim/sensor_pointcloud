/**
    sensor_pointcloud
    sensor_pointcloud_node.h
    Purpose: ROS Node for creating and managing sensor_pointcloud instances

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sensor_pointcloud/sensor_pointcloud.h>
#include <sensor_pointcloud/SensorPrecipitator.h>

using namespace sensor_pointcloud;

/**
    Entry point for ROS Node Execution

    @param argc number of arguments
    @param argv argument list
    @return exit status error code, or zero.
*/

int main(int argc, char** argv) {
    // ros::init must be the first call in main()
    ros::init(argc, argv, ROS_PREFIX);
    ros::NodeHandle nh, nhp("~");

    ROS_INFO("%s: v%s", ROS_PREFIX.c_str(), VERSION_STRING.c_str());

    // Remap and resolve Topic Names
    std::string pointcloudTopic = ros::names::append(ros::this_node::getName(), "pointcloud");
    if (ros::names::remap(pointcloudTopic) != pointcloudTopic) {
        pointcloudTopic = ros::names::remap(pointcloudTopic);
    }
    std::string parent_namespace = ros::names::parentNamespace(pointcloudTopic);
    ROS_INFO("Pointcloud Topic : %s", pointcloudTopic.c_str());

    // Read Parameters for Topics, Sensor Transforms, etc.
    std::string pointcloudFrame;
    nhp.param<std::string>(ros::names::append(parent_namespace, "pointcloudFrame"), pointcloudFrame, "map");
    ROS_INFO("Pointcloud Frame : %s", pointcloudFrame.c_str());

    // Create SensorPrecipitator Object
    SensorPrecipitator precipitator(pointcloudTopic, pointcloudFrame);

    ROS_INFO("SensorPrecipitator created");

    // Add Sensor Topics to SensorPrecipitator
    std::vector<std::string> sensors;
    nhp.getParam("sensors", sensors);
    if (sensors.size() == 0) ROS_WARN("No sensors configured");

    // For each detected sensor, read the parameters and initialize Sensor objects
    for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
        std::string sensorTopic, sensorFrame;
        nhp.getParam(*sensorNameIt + "/topic", sensorTopic);
        nhp.getParam(*sensorNameIt + "/transform/frame", sensorFrame);
        ROS_INFO("Sensor Parameters Loaded - Topic: %s Frame: %s", sensorTopic.c_str(), sensorFrame.c_str());

        boost::shared_ptr<Sensor> s = precipitator.addSensor(sensorTopic, sensorFrame);

        // Determine if transform is defined in yaml
        bool loadTransform = false;
        double translation[3] = {0, 0, 0};
        // Load x/y/z parameters
        for (char c = 'X'; c <= 'Z'; c++) {
            std::string paramStr = *sensorNameIt + "/transform/pos" + c;
            if (nh.getParam(paramStr, translation[c - 'X'])) {
                loadTransform = true;
                ROS_INFO("Loading Transform for %s: %f", paramStr.c_str(), translation[c-'X']);
            }
        }
        if (loadTransform) {
            // Load roll/pitch/yaw parameters - WILL NOT LOAD IF x/y/z not set
            tf2::Quaternion q;
            float roll = 0, pitch = 0, yaw = 0;
            nhp.getParam(*sensorNameIt + "/transform/roll", roll);
            nhp.getParam(*sensorNameIt + "/transform/pitch", pitch);
            nhp.getParam(*sensorNameIt + "/transform/yaw", yaw);
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
            ROS_INFO("Sensor Transform Loaded - Frame: %s (%.2f, %.2f, %.2f) rot: {%.2f, %.2f, %.2f}",
                     sensorFrame.c_str(), translation[0], translation[1], translation[2],
                     roll, pitch, yaw);
        }
    }

    ROS_INFO("Sensors added");

    // Process all event callbacks
    ros::spin();
}
