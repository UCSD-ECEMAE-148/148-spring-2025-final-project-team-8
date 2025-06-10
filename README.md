# 148-spring-2025-final-project-team-8
148-spring-2025-final-project-team-8 created by GitHub Classroom

To run the code:

- Start a container
- Clone into `ros2_ws/src/`
- Modify `ros2_ws/src/ucsdrobocar_hub2/ucsd_robocar_nav2_pkg/config/*` as necessary (actiave ld06, vesc_without_odom, DO NOT ACTIVATE OAKD)
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
