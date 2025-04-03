# ArUco-based positioner and box detector

## Preparing the system

### Downloading submodules

``` sh
git submodule update --init
```

### Host system dependencies

- podman
- distrobox

### Preparing the image

``` sh
podman build -t bananas-base:0.0.4 ./containers/aruco-base
```

### Preparing a container

``` sh
distrobox create --name bananas-aruco --image bananas-base:0.0.4
```

Add `--nvidia` to include support for NVIDIA's proprietary GPU driver.

## Building

Enter the container you created earlier:

``` sh
distrobox enter bananas-aruco
```

### Configuring

From inside the Distrobox container, run

``` sh
cmake -GNinja -S . -B build -DCMAKE_PREFIX_PATH='/usr/opencv;/usr/ogre;/usr/cv_bridge;/usr/mavlink' \
                            -DCMAKE_BUILD_TYPE=Release \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

If you want to include ROS support, run

``` sh
. /opt/ros/jazzy/setup.bash
```

to set up the ROS environment and then add `-DWITH_ROS2=ON` to the CMake command.

### Compilation

``` sh
cmake --build build
```

## Usage

### Positioner

The positioner requires three or four inputs depending on whether it was built
with ROS support:

- A JSON description of the boards
- A JSON description of the static environment
- A JSON file containing the camera parameters
- An input video to analyze if `WITH_ROS2` is not enabled

The file formats are currently not documented. An example configuration is
available at [Google
Drive](https://drive.google.com/drive/folders/1jW_gUaRNqzDQmUnwXLOY9ooAgiT-EK1z?usp=drive_link).

#### Non-ROS

``` sh
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json video.mp4
```

#### ROS

``` sh
. /opt/ros/jazzy/setup.bash
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json &
./build/apps/ros_video_publisher video.mp4
```

To use camera input
``` sh
. /opt/ros/jazzy/setup.bash
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json &
./build/apps/ros_video_publisher camera
```

### Gazebo simulation

#### General notes

- The Gazebo integration requires the project built with `-DWITH_ROS2=ON`.
- Gazebo must not be run from a shell session in which
  `/opt/ros/jazzy/setup.bash` has been sourced, or `gz` won't find the `sim`
  subcommand.
- Gazebo will freeze if you have are using a firewall that blocks multicast
  traffic. Example for temporarily allowing multicast on `firewalld`:

  ``` sh
  firewall-cmd --add-rich-rule='rule family=ipv4 destination address="224.0.0.0/4" accept'
  ```

#### box_world demo

##### Gazebo startup

In one shell instance, start up Gazebo with the main SDF file. Example, run from
the aruco source subdirectory in which this README is located:

``` sh
GZ_SIM_RESOURCE_PATH=$(pwd)/build/gazebo/box_world gz sim box_world.sdf
```

Unpause the simulation to make it start producing image data.

##### Positioner startup

In another shell instance, run inside your build directory:

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/box_world/model/camera/link/link/sensor/sensor/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/box_world/boards.json -env=gazebo/box_world/static_environment.json -camera=gazebo/box_world/camera.json \
                  --ros-args -r aruco_camera/image:=/world/box_world/model/camera/link/link/sensor/sensor/image --
```

#### drone_world demo

##### Component startup

###### Gazebo startup 

In one shell instance, start up Gazebo with the main SDF file. Example, run from
the aruco source subdirectory in which this README is located:

``` sh
GZ_SIM_RESOURCE_PATH=$(pwd)/build/gazebo/drone_world:$(pwd)/gazebo/PX4-gazebo-models/models gz sim drone_world.sdf
```

Remember to unpause the simulation before attempting to take off.

###### PX4 startup

After starting up Gazebo, run in another shell instance:

``` sh
PX4_SIM_MODEL=gz_x500_mono_cam_down /usr/px4/px4/bin/px4 /usr/px4/px4/etc
```

PX4 will store extra files to whatever directory you are in.

###### Positioner startup

In a yet another shell instance, run inside your build directory:

``` sh
. /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image &
./apps/positioner -boards=gazebo/drone_world/boards.json -env=gazebo/drone_world/static_environment.json -camera=gazebo/drone_world/camera.json -mavlink=udpin://0.0.0.0:14540 --ros-args -r aruco_camera/image:=/world/drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image --
```

##### Parameter setup

Set the following PX4 parameter values, for example using QGroundControl:

| Parameter | Value | Explanation |
| --------- | ----- | ----------- |
| SYS_HAS_MAG | 0 | Disable the magnetometer (requires restarting PX4) |
| EKF2_EV_CTRL | 11 | Use position and yaw data from our positioner |

Note that the parameters are persisted by PX4 in whatever directory you run it
from. If you want to reuse the same parameters later, run PX4 from the same
directory in which you set the parameters up earlier.

##### Global positioning

To allow using flight modes that depend on global position information, set an
origin for the local coordinate system. The PX4 console seems to be the most
reliable way to do this. Example PX4 console command:

```
commander set_ekf_origin 60.184933 24.828757 0.0
```

##### Taking off

Start up all the required software and connect QGroundControl to the drone (no
need to run it inside the container). Don't use QGroundControl's "Takeoff"
button to take off since it forces a minimum altitude of 10 meters. Instead, set
the `MIS_TAKEOFF_ALT` parameter if needed and use `commander takeoff` on the PX4
console if you want to perform a manual take off. If you want to fly a mission,
then starting the mission is enough to make the drone take off.
