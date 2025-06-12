# ECE 148 Spring 2025 Team 8 Final Project
## "Trojan Catapult"
![IMG_2721](https://github.com/user-attachments/assets/faab87ed-7164-49da-8889-461ea085e3fc)

[Final Presentation](https://docs.google.com/presentation/d/1iMlgrbB5Llozv7VuEhipj4CFqbd2fXouDEKI_ULL91k/edit?usp=sharing)

## Members
Alexandra Dinh -- Electrical Engineering

Hunter Hayes -- Mechanical Engineering

Sean Stillwell -- Mechanical Engineering

Michael Sullivan -- Mathematics: Computer Science

Elena Tomson -- Computer Science

## Description

Our initial goal for this project was to explore different options for robocar object detection and identify strengths and weaknesses in each. After a few weeks of exploration, we ultimately chose to narrow our scope and use our most promising options to create the Trojan Catapult. This project uses an image detection model paired with depth detection to locate a specific target, in this case a bucket, position itself accordingly, and fire a projectile, with the goal of showcasing the precision and efficiency of our chosen object detection pipeline.

### Initial Goals
We started this project with the hope of building off of the car following task from previous projects. Our goal was to perfect the object detection pipeline by exploring some of the different options available to us for object detection: image detection models, LLMs for image processing, image filtering techniques, stereo depth detection, and LiDAR fusion. Our later goal was to develop an all-in-one script that allowed the user to instruct the car to follow, avoid, or trace the movement of its target.

### Pivot Into Final Product
After a few weeks of exploration, we decided to narrow our deliverable to a more concrete task. Our goal became to implement our best-performing object detection pipeline to instruct the car to fire a projectile into a distant container. This allowed us to implement a mechanical aspect to our project and focus our efforts into a single well-performing task that can easily be generalized in the future. We ultimately discovered that the Roboflow depthai-sdk on OAK-D lite paired with LiDAR for depth detection provided a faithful understanding of immediate surroundings. Utilizing the depthai-sdk also gives us more flexibility with our model, allowing plug-and-play interractivity between the robocar and the Roboflow server. For this project, our model is simple bucket detection. Combined with the LiDAR's depth detection, these data provide the robocar with high-resolution awareness of the target at a range of up to 3 meters.

### Future Goals
1. Due to time and material constraints, our catapult does have several variables leading to potential inconsistencies with range, distance, and spread, which made calibrating the car around it challenging. We would like to either replace the catapult mechanism with something more reliable, or refine the structure and material to provide more consistent results.
2. We would like to continue developing the car's capabilities beyond stationary targeting, including tracking and following.

# Docs
To run the code:
- Start a docker container
```
docker run \
    --name my_robocar_container \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --device /dev/video0 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    djnighti/ucsd_robocar:devel
```
- Install [depthai-sdk](https://github.com/luxonis/depthai/tree/main)
- Clone our repository into `ros2_ws/src/`
- Modify `ros2_ws/src/ucsdrobocar_hub2/ucsd_robocar_nav2_pkg/config/*` as necessary
   - activate ld06, vesc_without_odom, DO NOT ACTIVATE OAKD
   - update package location with team8_pkg
   - update node config to launch team8_pkg.launch.py
- Run the following: `source_ros2`, `build_ros2`
- To launch: `ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py`
  - After starting all nodes, should receive a "Successfully loaded model" message from target-node
  - Terminal should be printing "Waiting for launch"
- Open secondary terminal to publish strings to the `/cin` (control input) topic
  - Manual Reload (set the servo in the "primed" position) :
    
    `ros2 topic pub --once /cin std_msgs/String "data: 'reload'"`
  - Manual Trigger (set the servo in the "resting" position, usually sent after reloading) :
    
    `ros2 topic pub --once /cin std_msgs/String "data: 'shoot'"`
  - Launch Robot (tells robocar to begin searching and positioning itself from the target. usually sent after reloading) :

    `ros2 topic pub --once /cin std_msgs/String "data: 'launch'"`

# Files of Note: 
#### `lane_detection_node.py`
- subscribes to `/centroids` and `/scan` for bounding boxes and laser scan data
- publishes to `/location` with error value and depth
- `LABEL` controls the targeted label. `0` for Blue bucket, `1` for Orange bucket
- `CAM_CENTER` <=1 controls the "center" of the camera. Ex. `0.5` is the direct center of the frame. Can be modified if the catapult has a tendency to not aim directly straight

### `lane_guidance_node.py`
- subscribes to `/location` for error value and depth data, `/cout` for feedback from control node
- publishes to `/cmd_vel` for vesc control, `/cin` for sending commands to control node (firing the catapult)
- `THRESHOLD_DIST` (meters) controls the desired target distance. The car will do its best to achieve this distance from the target
- `THRESHOLD_VAR` (meters) controls the allowed variance from the target distance. The car will consider any value within `THRESHOLD_DIST + or - THRESHOLD_VAR` as "in range"
- `THRESHOLD_VAR_FINE` (meters） controls the distance from `THRESHOLD_DIST` in which the car will activate fine control mode
- `LOCKON` (seconds) controls the amount of time the car will wait while being centered and in-range before firing the catapult

### `target_node.py`
- publishes to `/centroids` with bounding box data (label and centroid)
- `MODEL_NAME` is the roboflow model. Swap this out if you want to use a different model

### `control_node.py`
- subscribes to `/cin` (control input) and publishes to `/cout` (control output)
- is also the node controlling the trigger servo
