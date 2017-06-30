/**
    sensor_pointcloud
    sensor_pointcloud.h
    Purpose: Include common headers and constants

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SENSOR_POINTCLOUD_H
#define SENSOR_POINTCLOUD_H

#include <string>
#include <vector>
#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

namespace sensor_pointcloud {
    const std::string ROS_PREFIX = "sensor_pointcloud";
    const std::string VERSION_STRING = "0.1.4";
}

#endif
