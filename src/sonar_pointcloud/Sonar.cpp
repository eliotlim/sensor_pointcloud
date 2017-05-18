/**
    Sonar Class
    Sonar.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/Sonar.h>

using namespace sonar_pointcloud;

Sonar::Sonar(std::string sonarTopic, std::string sonarFrame) :
             topic(sonarTopic), frame(sonarFrame) {
    // ROS Setup
    nodeHandle = ros::NodeHandle();
    rangeSubscriber = nodeHandle.subscribe(sonarTopic, 20, &Sonar::rangeCallback, this);

    transform = false;
}

/**
    Set Transform
    Store Stamped Transform for broadcast
    Will automatically provide child frame

    @param TransformStamped
*/

void Sonar::setTransform(geometry_msgs::TransformStamped transformS) {
    transform = true;
    this->transformS = boost::shared_ptr<geometry_msgs::TransformStamped>(new geometry_msgs::TransformStamped(transformS));
    this->transformS->child_frame_id = frame;
}

geometry_msgs::TransformStamped Sonar::getTransform() { return *transformS; }

/**
    Range Message Callback
    Process and Store (by copy) received range

    @param range_msg Sonar range message
*/

void Sonar::rangeCallback(const sensor_msgs::Range& range_msg) {
    if (frame.compare(range_msg.header.frame_id) != 0) {
        this->range_msg = boost::shared_ptr<sensor_msgs::Range>(new sensor_msgs::Range(range_msg));
    }
}

/**
    getRange
    Return the range from most recent range_msg

    @return range
*/

float Sonar::getRange() {
    if (!range_msg) return -1;
    if (range_msg->range < range_msg->min_range || range_msg->range > range_msg->max_range) {
        return -1;
    }
    return range_msg->range;
}
