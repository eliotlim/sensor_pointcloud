# Configuring sensor_pointcloud

## Using sensorConfig.yml

### Structure
#### Header
```
%YAML 1.2
---
# Configuration
sensor_pointcloud:
    pointcloudFrame: map
```
This portion declares:
1. The configuration file format (YAML 1.2)
2. The transform frame for the pointcloud

#### List of data sources
```
# List of Sensors
sensors:
    - sensor0
    ...
    - sensorX
```

#### Details for each data source
```
sensor0:
    topic: ultrasound/range
    transform:
        frame: sensor0
        posX: 0.03
        posY: -0.07
        posZ: 0.05
        roll: deg(0)
        pitch: deg(0)
        yaw: deg(-90)
```
Break it down.
  1. Identifier `sensor0` as defined in the list of data sources
  2. Topic on which the device broadcasts `sensor_msgs::Range` messages
  3. (Optional) Transform of the sensor with respect to `pointcloudFrame`
