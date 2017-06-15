# Configuring sonar_pointcloud

## Using sonarConfig.yml

### Structure
#### Header
```
%YAML 1.2
---
# Configuration
sonar_pointcloud:
    pointcloudFrame: map
```
This portion declares:
1. The configuration file format (YAML 1.2)
2. The transform frame for the pointcloud

#### List of data sources
```
# List of Sonars
sonars:
    - sonar0
    ...
    - sonarX
```

#### Details for each data source
```
sonar0:
    topic: ultrasound/range
    transform:
        frame: sonar0
        posX: 0.03
        posY: -0.07
        posZ: 0.05
        roll: deg(0)
        pitch: deg(0)
        yaw: deg(-90)
```
Break it down.
  1. Identifier `sonar0` as defined in the list of data sources
  2. Topic on which the device broadcasts `sensor_msgs::Range` messages
  3. (Optional) Transform of the sensor with respect to `pointcloudFrame`
