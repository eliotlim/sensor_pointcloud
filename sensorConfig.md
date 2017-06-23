# Configuring sensor_pointcloud

## Using sensorConfig.yml

### Structure
#### Header
```yaml
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
```yaml
# List of Sensors
sensors:
    - sensor0
    ...
    - sensorX
```

#### Details for each data source
```yaml
sensor0:
    topic: ultrasound/range
    device:
        type: SRF05
        pin: 14
        timeout: 60000
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
  - Identifier `sensor0` as defined in the list of data sources
  - Topic on which the device broadcasts `sensor_msgs::Range` messages
  - (Optional) Sensor Definition for use with [sensor_hub](https://github.com/eliotlim/sensor_hub) package
  - (Optional) Transform of the sensor with respect to `pointcloudFrame`
