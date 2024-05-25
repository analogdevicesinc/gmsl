# ROS2 support for gmsl video decoding over UDP/RTP

This provide ROS2 node for receiving and decoding RTP packets from gmsl video transmitter.

### Prerequisites
#### Install ROS Humble
Follow the instructions on the [ROS Humble Installation Page](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS Humble on your machine.

#### Install OpenCV and cv_bridge

The video_receiver package depends on OpenCV:
```bash
sudo apt-get install libopencv-dev
```

cv_bridge is a ROS package that provides an interface between ROS and OpenCV. It is part of the vision_opencv stack.

```bash
sudo apt-get install ros-humble-cv-bridge
sudo apt-get install ros-humble-vision-opencv
```
#### Install Boost Libraries

Boost.Asio is used for network and low-level I/O programming. It provides the means to create a UDP socket, which is used to receive the video data.

```bash
sudo apt-get install libboost-all-dev
```

#### Install GStreamer for testing on PC camera dev

You need to have GStreamer installed on your machine. You can install it using the following command:

```bash
sudo apt-get install gstreamer1.0-tools
```

### Building and Running the video_receiver Package

Checkout the `ros_rtp` branch from your repository:

```bash
git checkout ros_rtp
cd ros_ws
```
Source the ROS setup script:

```bash
source /opt/ros/humble/setup.bash
```
Build the workspace using `colcon`:

```bash
colcon build
```

Source the setup script for the built packages:

```bash
source install/local_setup.bash
```

Run the `video_receiver` node:

```bash
ros2 run video_receiver video_receiver
```

Or, launch the `rtp_video_launch` launch file:

```bash
ros2 launch video_receiver rtp_video_launch.py
```
#### Adding Multiple Nodes

You can add multiple nodes in the launch file to listen to different ports. Each node represents a separate video stream. Here is an example of a node:

```python
Node(
    package='video_receiver',
    executable='video_receiver',
    name='rtp_video_receiver_node',
    parameters=[
        {'ip': '127.0.0.1'},
        {'port': 5004},
        {'topic': 'cam0'},
        {'width': 1280},
        {'height': 720}
    ]
)
```

### Running the gsteamer transmitter over udp

#### Checking Camera Resolution

Before streaming video, you may want to check the supported resolutions of your camera. You can do this using the `v4l2-ctl` command:

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```
#### Start Gstreamer transmitter
Testing can be done using video input from web camera (/dev/video0). To start streaming video from your web camera to UDP port 5004, run the following command:

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! rtpvrawpay mtu=16334 ! udpsink host=127.0.0.1 port=5004
```

To start streaming video on multiple UDP ports: 5004, 5005, 5006, and 5007, run the following command:

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! rtpvrawpay mtu=16334 ! tee name=t t. ! queue ! udpsink host=127.0.0.1 port=5004 t. ! queue ! udpsink host=127.0.0.1 port=5005 t. ! queue ! udpsink host=127.0.0.1 port=5006 t. ! queue ! udpsink host=127.0.0.1 port=5007
