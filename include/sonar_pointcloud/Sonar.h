/**
    Sonar Class
    Sonar.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SONAR_H
#define SONAR_H

#include "sonar_pointcloud.h"

#include <sensor_msgs/Range.h>

namespace sonar_pointcloud {

class Sonar {
public:
    Sonar(const std::string& sonarTopic, const std::string& sonarFrame);
    ~Sonar() {}

    void rangeCallback(const sensor_msgs::Range& range_msg);
    float getRange();

    std::string topic;
    std::string frame;

private:
    ros::Subscriber rangeSubscriber;
    sensor_msgs::Range range_msg;

    ros::NodeHandle nodeHandle;
};

}

#endif
