/**
    SensorPrecipitator Class
    SensorPrecipitator.h
    Purpose: Class that converts `range msg` and `transforms` into PointCloud2
    Also publishes sensor frame transforms if defined.

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SONAR_PRECIPITATOR_H
#define SONAR_PRECIPITATOR_H

#include "sensor_pointcloud.h"
#include "sensor_pointcloud/Sensor.h"

#include <boost/thread.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace sensor_pointcloud {

class SensorPrecipitator {
public:
    SensorPrecipitator(const std::string pointcloudTopic, const std::string pointcloudFrame);
    ~SensorPrecipitator() {}
    boost::shared_ptr<Sensor> addSensor(const std::string sensorTopic, const std::string sensorFrame);

private:
    void publishCallable();

    ros::NodeHandle nodeHandle;
    ros::Publisher pointcloudPublisher;
    boost::thread publishThread;
    double publishRate;

    std::string frame;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::StaticTransformBroadcaster tfBroadcaster;

    std::vector<boost::shared_ptr<sensor_pointcloud::Sensor> > sensors;

};

}

#endif
