/**
    Sonar Class
    Sonar.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/Sonar.h>

using namespace sonar_pointcloud;

/**
    Constructor for Sonar
    Instantiates topic and frame, and subscribes to the relevant topics

    @param sonarTopic topic for sonar data
    @param sonarFrame TF2 Frame for sonar sensor
*/

Sonar::Sonar(std::string sonarTopic, std::string sonarFrame) :
             topic(sonarTopic), frame(sonarFrame) {
    // ROS Setup
    nodeHandle = ros::NodeHandle();
    rangeSubscriber = nodeHandle.subscribe(sonarTopic, 20, &Sonar::rangeCallback, this);

    transform = false;
}

/**
    Set Transform
    Store (by copy) Stamped Transform for broadcast
    Will automatically set child frame

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
    // Check if frame matches range_msg frame_id
    if (frame.compare(range_msg.header.frame_id) == 0) {
        this->range_msg = boost::shared_ptr<sensor_msgs::Range>(new sensor_msgs::Range(range_msg));
        ROS_DEBUG("Received range_msg - frame: %s range: %f", frame.c_str(), range_msg.range);
    }
}

/**
    getRange
    Return the range from most recent range_msg

    @return range
*/

float Sonar::getRange() {
    // Check range_msg not NULL and min_range/max_range bounds not exceeded
    if (!range_msg) return -1;
    if (range_msg->range < range_msg->min_range || range_msg->range > range_msg->max_range) {
        return -1;
    }
    return range_msg->range;
}
