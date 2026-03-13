# Human Pose Estimation Node

**Harokopio University of Athens**

**Contact info: kfoteinos@hua.gr**

### General information

This module takes as input aligned RGBD frames and the (global) pose of the camera (i.e. orientation and 2D position) at that timestamp (approximately) and obtains, for every visible human, the pixel coordinates (uv) of her/his keypoints (e.g. ankles, shoulders), their depth and estimation confidence. Further, it estimates the uv position and depth of each human by getting the average of the per-keypoint uv and depth. This information is utilized to predict the (global) longitude and latitude coordinates.

### Provided interface

| Topic name | Message type | Usage | Details |
| --- | --- | --- | --- |
| /camera_front/color | Image | Input | 8-bit RGB (H x W x 3) |
| /camera_front/depth | Image | Input | 16UC1 in mm (H x W x 2) aligned to the RGB |
| /camera_front/camera_info | CameraInfo | Input | - |
| /fix | NavSatFix | Input | - |
| /dog_odom | Odometry | Input | Orientation should be expresses wr.t. to a global coordinate system (the "standard" xy plane aligned to parallels and meridians) |
| /human_pose | String | Output | Stringified GeoJSON, see below |

The JSON provides the position of the human(s) (by averaging the position of the keypoints), not the position of each one keypoint. In particular, the "features" field of the published message has the following format:

```json
[
    {
        "type": "Feature",
        "geometry": {
            "type": "Point",
            "coordinates": [ <lon>, <lat> ]
        },
        "properties": {
            "depth": <average depth in mm>,
            "timestamp": <timestamp>,
            "keypoints_and_depths": {
                <keypoint name>: [ <u coordinate in pixels>, <v coordinate in pixels>, <confidence>, <depth in mm> ],
                ...
            }
        },
        "relative_position":[ <w.r.t. to camera coordinate system> ]
    },
    for every detected person
    ...
]
```

> Zero depth means u or v exceeds the limits of the depth frame, i.e. the detection falls out the image

See also the example below, produced by the command `ros2 topic echo /human_pose --once --full`.

```json
{
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [
                    0.02245154528998048,
                    0.0
                ]
            },
            "properties": {
                "depth": 1661.0588235294117,
                "timestamp": 1773353514330361862,
                "keypoints_and_depths": {
                    "Nose": [
                        280,
                        164,
                        0.9914366602897644,
                        1935.0
                    ],
                    "Left Eye": [
                        289,
                        156,
                        0.9917678833007812,
                        1949.0
                    ],
                    "Right Eye": [
                        272,
                        156,
                        0.9940339922904968,
                        1953.0
                    ],
                    "Left Ear": [
                        303,
                        167,
                        0.8557296991348267,
                        2044.0
                    ],
                    "Right Ear": [
                        260,
                        166,
                        0.861356258392334,
                        1975.0
                    ],
                    "Left Shoulder": [
                        321,
                        214,
                        0.9966464638710022,
                        1960.0
                    ],
                    "Right Shoulder": [
                        242,
                        209,
                        0.9963833093643188,
                        1971.0
                    ],
                    "Left Elbow": [
                        358,
                        152,
                        0.9968279004096985,
                        1910.0
                    ],
                    "Right Elbow": [
                        208,
                        154,
                        0.9867075681686401,
                        1903.0
                    ],
                    "Left Wrist": [
                        311,
                        90,
                        0.991696298122406,
                        1886.0
                    ],
                    "Right Wrist": [
                        242,
                        104,
                        0.9797500371932983,
                        1903.0
                    ],
                    "Left Hip": [
                        316,
                        372,
                        0.9914780259132385,
                        1727.0
                    ],
                    "Right Hip": [
                        257,
                        369,
                        0.9926723837852478,
                        1659.0
                    ],
                    "Left Knee": [
                        333,
                        479,
                        0.2418973743915558,
                        1727.0
                    ],
                    "Right Knee": [
                        249,
                        473,
                        0.30138349533081055,
                        1736.0
                    ],
                    "Left Ankle": [
                        340,
                        480,
                        0.0023411947768181562,
                        0
                    ],
                    "Right Ankle": [
                        234,
                        480,
                        0.0016549858264625072,
                        0
                    ]
                },
                "relative_position": [
                    11955.743647058822,
                    10176.160588235294,
                    1660.0588235294117
                ]
            }
        }
    ]
}
```

### Instructions for setup (docker)

To build the image:
```bash
sudo docker build -t human_pose_container .
```

> To view the existing images:
> ```bash
> sudo docker images -a
> ```
>
> To remove an image:
> ```bash
> sudo docker rmi human_pose_container
> ```

To create & enter the container (the `--net host` is mandatory to enable access to/from outside topics):
```bash
sudo docker run --net host -it human_pose_container /bin/bash
```

If GPU access is available:
```bash
sudo docker run --runtime=nvidia --net host -it human_pose_container /bin/bash
```

To view the running containers:
```bash
sudo docker container ps -a
```

> To remove all containers:
> ```bash
> sudo docker container prune
> ```

To enter the container:
```bash
sudo docker exec -it <container ID> bash
```

### Instructions for setup (run the given bag)

Extract everything from the given .zip file and run:
```bash
ros2 bag play <mcap file> --loop
```

### Instructions for setup (run ROS)

After entering the container:

Activate ROS (Humble) (zsh, bash, ... according to your terminal):
```bash
source /opt/ros/humble/setup.bash
```

Build the package:
```bash
colcon build --packages-select pose_estimation
```

Run the following before use the package:
```bash
source ./install/local_setup.bash
```

Run the node:

```bash
ros2 run pose_estimation estimator
```

View the detections topic:
```bash
ros2 topic echo human_pose --once --full
```
