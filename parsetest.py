import gi
# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib

Gst.init()
launch = 'input-selector name="selector" !  \
    nvvidconv name=converter ! video/x-raw(memory:NVMM), width=(int)5400, height=(int)1920, format=(string)I420 ! \
    nvv4l2h265enc name=encoder ! video/x-h265, stream-format=(string)byte-stream ! h265parse ! \
    rtph265pay config-interval=1 name=pay0 pt=96 \
    rtspsrc location="rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101" name=source0 latency=100 ! queue2 ! rtph265depay ! queue ! nvv4l2decoder ! selector.sink_0 \
    videotestsrc pattern=21 kt=8 ! video/x-raw, width=(int)5400, height=(int)1920, format=(string)I420 ! timeoverlay  ! selector.sink_1'
Gst.parse_launch(launch)