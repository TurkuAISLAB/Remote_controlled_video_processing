# Python based RTSP server for video transcoding and selection wtih ROS2 control


## py_rtsp_server
ROS2 Python module that starts a ROS2 node exposing two ROS parameters, bitrate and selection. Does not currently set startup values based on the parameter values so initial pipeline settings cannot be changed with `ros2 launch` changing parameters.

#### Building and running
To run with `ros2 run`, do `colcon build`. Make sure that Gstreamer dependencies are installed by installing the GObject introspection `gir1.2-gst-rtsp-server-1.0` and Python GObject introspection `python3-gi` packages with `apt install`. Gstreamer is not included in the ROS package index and can not be installed with `rosdep`. These packages form the Python bindings for the Gstreamer RTSP server library and will pull other Gstreamer bindings and libraries as well. The exact `gir` version number may change between linux versions but for Ubuntu 22.04 and 24.04 it is 1.2.   

#### Operation
After a client connects to the RTSP endpoint and begins receiving video changes to the ROS parameters made with `ros2 param set {value}` will be reflected in the video output if possible and return a `Parameter change successful` message to the terminal that issued the param set command. If the pipeline does not accept the change a `Parameter change unsuccesful` message is returned instead.

The selection parameter is an integer, indexing between the various inputs linked to the `inputselector` element in the pipeline with the property `name="selector"`. Indexing begins at 0 so with two input you would set selection to either 0 or 1. Anything else will return a parameter change failed.

The bitrate parameter is an integer representing the target bitrate of the encoder element in the pipeline with the property `name="encoder"`. The encoding mode (usually set with the `mode` property) needs to be set to Constant-Bitrate (`cbr`) mode. Depending on encoder used this value may represent bitrate in either bits/s or kbit/s. Use `gst-inspect-1.0 [encodername]` to view the encoder's included documentation to check which type the selected encoder uses


#### Extra

An incompatibility exists between Gstreamer and the ROS2 Python runtime library due to an erroneous symbol export of `libunwind` symbols, causing a segfault on startup. By setting the environment variable `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8` this error is eliminated and only needs to be done for the process that launches the ROS2 node. This is potentially fixed on Ubuntu 24.04.

Current version uses software encoder (x264) and Gstreamer test video sources (VA-API was somewhat unreliable) but commented out pipeline using Nvidia Jetson based modules is also present in code. Comment out existing pipeline and uncomment pipeline that inculdes `nvv4l2h265dec`, `nvv4l2h265enc` and `nvvidconv` elements.

## inputselector.py

Test application for rapidly testing Gstreamer pipelines. Includes a GUI with buttons to increase/decrease encoder bitrate and switch between inputs.

Currently set with a pipeline that reads two local files (not included in repository)

## rtspserver.py
Barebones RTSP server to serve video from files on the computer over RTSP to clients. For testing `rtspsrc` elements of Gstreamer pipelines.