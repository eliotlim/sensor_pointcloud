/**
    Sonar Class
    Sonar.h
    Purpose: Class that subscribes to `range_msg`

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sonar_pointcloud/Sonar.h>

using namespace sonar_pointcloud;

Sonar::Sonar(const std::string& sonarTopic, const std::string& sonarFrame) :
             sonarTopic(sonarTopic), sonarFrame(sonarFrame) {
    // ROS Setup
    nodeHandle = ros::NodeHandle();
    rangeSubscriber = nodeHandle.subscribe(sonarTopic, 20, &Sonar::rangeCallback, this);
}

/**
    Range Message Callback
    Process and Store (by copy) received range

    @param range_msg Sonar range message
*/

void Sonar::rangeCallback(const sensor_msgs::Range& range_msg) {
    if (sonarFrame.compare(range_msg.header.frame_id) != 0) {
        this->range_msg = range_msg;
    }
}

float Sonar::getRange() {
    if (range_msg.range < range_msg.min_range || range_msg.range > range_msg.max_range) {
        return -1;
    }
    return range_msg.range;
}
