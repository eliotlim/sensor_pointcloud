/**
    sonar_pointcloud
    sonar_pointcloud.h
    Purpose: Include common headers and constants

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#ifndef SONAR_POINTCLOUD_H
#define SONAR_POINTCLOUD_H

#include <string>
#include <vector>
#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

namespace sonar_pointcloud {
    const std::string ROS_PREFIX = "sonar_pointcloud";
    const std::string VERSION_STRING = "0.1.0";
}

#endif
