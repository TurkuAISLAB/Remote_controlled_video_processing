#! usr/bin/bash

# rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101
# rtsp://admin:L48yr1n771@192.168.128.191:554/Streaming/Channels/101

# NOTE: Do NOT put a h265parse before a nvv4l2decoder with output straight from the camera

#Panorama
gst-launch-1.0 \
    rtspsrc location='rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101' latency=100 ! queue2 ! \
    rtph265depay ! queue ! nvv4l2decoder ! nv3dsink

#Fisheye
gst-launch-1.0 \
    rtspsrc location='rtsp://admin:L48yr1n771@192.168.128.191:554/Streaming/Channels/101' latency=100 ! queue2 ! \
    rtph264depay ! queue ! nvv4l2decoder ! nv3dsink

#UDP sink transmitter
gst-launch-1.0 \
    videotestsrc ! 'video/x-raw, format=(string)I420, width=(int)8160, height=(int)2304' ! \
    nvvidconv ! 'video/x-raw(memory:NVMM), width=(int)4160, height=(int)1920, format=(string)I420' ! \
    nvv4l2h265enc ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! \
    rtph265pay pt=96 ! udpsink host=127.0.0.1 port=8001 sync=false

#UDP sink transmitter, start second, set receiving computer's IP address
gst-launch-1.0 \
    rtspsrc location='rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101' latency=100 ! queue2 ! \
    rtph265depay ! queue ! nvv4l2decoder ! \
    nvvidconv ! 'video/x-raw(memory:NVMM), width=(int)5400, height=(int)1920, format=(string)I420' ! \
    nvv4l2h265enc ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! \
    rtph265pay pt=96 ! udpsink host=127.0.0.1 port=8001 sync=false

#UDP src reciever, start first, set receiving computer's IP address 
gst-launch-1.0 udpsrc address=127.0.0.1 port=8001 \
     caps='application/x-rtp, encoding-name=(string)H265, payload=(int)96' ! \
     rtph265depay ! queue ! h265parse ! nvv4l2decoder ! nv3dsink

gst-launch-1.0 \
    input-selector name="selector" ! rtph265depay ! queue ! nvv4l2decoder ! \
    nvvidconv ! "video/x-raw(memory:NVMM), width=(int)5400, height=(int)1920, format=(string)I420" ! \
    nvv4l2h265enc name=encoder ! "video/x-h265, stream-format=(string)byte-stream" ! h265parse ! \
    rtph265pay config-interval=1 name=pay0 pt=96 ! udpsink host=192.168.128.236 port=8001 sync=false \
    rtspsrc location="rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101" latency=100 ! queue2 ! selector.sink_0

gst-launch-1.0 filesrc location="bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! vaapih265dec ! vaapisink

gst-launch-1.0 filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! vaapih265dec \
    ! vaapipostproc ! vaapih265enc name="encoder" ! vaapih265dec ! vaapisink name=output \

gst-launch-1.0 input-selector name="selector" ! vaapih265dec ! vaapipostproc ! vaapih265enc name="encoder" ! \
        vaapih265dec ! vaapisink name=output \
        filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! selector.sink_0 \
        filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! selector.sink_1