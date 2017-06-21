/**
    Sensor Class
    Sensor.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SENSOR_H
#define SENSOR_H

#include "sensor_pointcloud.h"

#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>

namespace sensor_pointcloud {

class Sensor {
public:
    Sensor(std::string sensorTopic, std::string sensorFrame);
    //~Sensor() {}

    void rangeCallback(const sensor_msgs::Range& range_msg);
    void setTransform(const geometry_msgs::TransformStamped transformS);
    boost::shared_ptr<geometry_msgs::TransformStamped> getTransform();
    float getRange();

    std::string topic;
    std::string frame;

    bool transform;

private:
    ros::Subscriber rangeSubscriber;
    boost::shared_ptr<sensor_msgs::Range> range_msg;
    boost::shared_ptr<geometry_msgs::TransformStamped> transformS;

    ros::NodeHandle nodeHandle;
};

}

#endif
