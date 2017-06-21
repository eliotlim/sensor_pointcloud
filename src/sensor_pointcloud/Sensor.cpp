/**
    Sensor Class
    Sensor.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sensor_pointcloud/Sensor.h>

using namespace sensor_pointcloud;

/**
    Constructor for sensor
    Instantiates topic and frame, and subscribes to the relevant topics

    @param sensorTopic topic for sensor data
    @param sensorFrame TF2 Frame for sensor sensor
*/

Sensor::Sensor(std::string sensorTopic, std::string sensorFrame) :
             topic(sensorTopic), frame(sensorFrame) {
    // ROS Setup
    nodeHandle = ros::NodeHandle();
    rangeSubscriber = nodeHandle.subscribe(sensorTopic, 20, &Sensor::rangeCallback, this);

    transform = false;
}

/**
    Set Transform
    Store (by copy) Stamped Transform for broadcast
    Will automatically set child frame

    @param TransformStamped
*/

void Sensor::setTransform(geometry_msgs::TransformStamped transformS) {
    transform = true;
    this->transformS = boost::shared_ptr<geometry_msgs::TransformStamped>(new geometry_msgs::TransformStamped(transformS));
    this->transformS->child_frame_id = frame;
}

boost::shared_ptr<geometry_msgs::TransformStamped> Sensor::getTransform() { return transformS; }

/**
    Range Message Callback
    Process and Store (by copy) received range

    @param range_msg sensor range message
*/

void Sensor::rangeCallback(const sensor_msgs::Range& range_msg) {
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

float Sensor::getRange() {
    // Check range_msg not NULL and min_range/max_range bounds not exceeded
    if (!range_msg) return -1;
    if (range_msg->range < range_msg->min_range || range_msg->range > range_msg->max_range) {
        return -1;
    }
    return range_msg->range;
}
