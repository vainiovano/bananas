### Running on Jetson with VNC and camera input

The following instructions describe how to run the positioner on a Jetson device using a VNC session and live camera input via ROS.

#### Starting a VNC session

SSH into the jetson device

```sh
vncserver
```

This will start a new session, usually available at :1. In a VNC viewer on your local machine, connect to:

```
{ip}:1
```

Replace `{ip}` with the actual IP address of the Jetson device.

#### Entering the container

```sh
distrobox enter bananas-base-2
```

#### Inside the container, set the display to the VNC session:

```sh
export DISPLAY=:1
```

This allows the positioner to render within the VNC session.

#### Source the ROS 2 setup script:

```sh
. /opt/ros/jazzy/setup.bash
```

#### Running the applications

Navigate to the source directory of the project:

Start the ROS video publisher using the Jetson's camera: at /dev/video0

The video publisher expects the pixel format to be MONO16 ('Y16 ' corresponds to MONO16) and video to be 1280x720
These can be edited, but Jetson's processing power cant handle 1920x1080 video reliably
```sh
v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1280,height=720,pixelformat='Y16 '
```

```sh
./build/apps/ros_video_publisher camera /dev/video0 &
```
& sends the process to background. It can be brought to foreground with
```sh
fg
```

Then run the positioner:

```sh
./build/apps/positioner -boards=boards.json -env=static_environment.json -camera=camera.json
```

This setup allows live feed from the positioner on Jetson, rendered inside the VNC session.
