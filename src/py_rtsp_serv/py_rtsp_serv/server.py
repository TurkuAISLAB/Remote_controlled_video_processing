import argparse
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

from threading import Thread

import gi
# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib

# Sensor Factory class which inherits the RTSPMediaFactory base class and add
# properties to it.
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.selector = None
        self.encoder = None
        # self.launch_string = 'input-selector name="selector" sync-mode=1 ! timeoverlay ! \
        #                       videoconvert ! video/x-raw,format=I420  \
        #                      ! x264enc name=encoder speed-preset=ultrafast tune=zerolatency  \
        #                      ! rtph264pay config-interval=1 name=pay0 pt=96 \
        #                       videotestsrc pattern=21 kt=8 ! selector.sink_0 \
        #                       videotestsrc pattern=0 ! selector.sink_1 '
    #     self.launch_string = 'input-selector name="selector" sync-mode=1 sync-streams=false ! rtph265depay ! queue ! nvv4l2decoder ! \
    # nvvidconv name=converter ! capsfilter name="size" caps="video/x-raw(memory:NVMM), width=(int)4320, height=(int)1920, format=(string)I420" ! \
    # nvv4l2h265enc control-rate=1 ratecontrol-enable=true name="encoder" ! video/x-h265, stream-format=(string)byte-stream ! h265parse ! \
    # rtph265pay config-interval=1 name=pay0 pt=96 \
    # rtspsrc location="rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101" name=source0 latency=100 ! queue2 ! selector.sink_0 \
    # rtspsrc location="rtsp://admin:L48yr1n771@192.168.128.190:554/Streaming/Channels/101" name=source1 latency=100 ! queue2 ! selector.sink_1'
        self.launch_string = f'input-selector name="selector" ! capsfilter caps="video/x-raw, height=600, width=800, framerate=30/1" ! \
                                          nvvidconv name=converter ! capsfilter name="size" caps="video/x-raw(memory:NVMM), width=(int)800, height=(int)800, format=(string)I420" !\
                                          nvv4l2h265enc name="encoder" control-rate=1 ratecontrol-enable=true bitrate=200000 ! h265parse ! \
                                          avdec_h265 ! videoconvert ! xvimagesink name=output \
                                          videotestsrc pattern=21 kt=8 is-live=true ! selector.sink_0 \
                                          videotestsrc pattern=0 is-live=true ! selector.sink_1'
        self.set_shared(True)
        self.own_media = None

    # attach the launch string to the override method
    def do_create_element(self, url):
        elem = Gst.parse_launch(self.launch_string)
        return elem
    
    # set configurable properties on the constructed media object
    def do_configure(self, rtsp_media):
        self.media = rtsp_media
        self.number_frames = 0
        self.selector = rtsp_media.get_element().get_child_by_name('selector')
        self.encoder = rtsp_media.get_element().get_child_by_name('encoder')
        print(self.is_shared())
        print(rtsp_media.is_shared())
        rtsp_media.set_shared(True)
        # rtsp_media.set_reusable(True)

    def switch_to_input(self, input):
        if self.selector != None:
            num_pads = self.selector.get_property('n-pads')
            print(f'Numer of inputs: {num_pads}')
            if (input < 0) or (input > (num_pads-1)):
                print('Out of bounds input selected')
                return False
            active_pad = self.selector.get_property('active-pad')
            if active_pad:
                active_pad_name = active_pad.get_name()
                print(active_pad_name)
                if active_pad_name != f'sink_{input}':
                    padname = f'sink_{input}'
                else:
                    return True
                newpad = self.selector.get_static_pad(padname)
                print(padname)
                print(newpad)
                self.selector.set_property('active-pad', newpad)
                print('set pad')
                return True
    def change_bitrate(self, bitrate):
        if self.encoder != None:
            elem = self.media.get_element()
            print(elem)
            print(self.encoder)
            self.encoder.set_property('bitrate', bitrate)
            return True

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.factory.latency = 100
        self.set_service(str(opt.port))
        self.get_mount_points().add_factory(opt.stream_uri, self.factory)

        # Create permissions for authenticated users
        permissions = GstRtspServer.RTSPPermissions.new()
        permissions.add_role('admin')
        permissions.add_permission_for_role('admin', GstRtspServer.RTSP_PERM_MEDIA_FACTORY_ACCESS, True)
        permissions.add_permission_for_role('admin', GstRtspServer.RTSP_PERM_MEDIA_FACTORY_CONSTRUCT, True)
        self.factory.set_permissions(permissions)

        # Create user authentication
        auth = GstRtspServer.RTSPAuth.new()
        anontoken =  GstRtspServer.RTSPToken.new()
        anontoken.set_string(GstRtspServer.RTSP_TOKEN_MEDIA_FACTORY_ROLE, "anonymous")
        token =  GstRtspServer.RTSPToken.new()
        token.set_string(GstRtspServer.RTSP_TOKEN_MEDIA_FACTORY_ROLE, "admin")
        basic = auth.make_basic('admin', 'admin')
        auth.add_basic(basic, token)
        self.set_auth(auth)
        
        # Attach server to default GLib MainContext
        self.attach(None)
        print('Server attached')

# getting the required information from the user 
parser = argparse.ArgumentParser()
parser.add_argument("--port", default=8554, help="port to stream video", type = int)
parser.add_argument("--stream_uri", default = "/video_stream", help="rtsp video stream uri")
opt = parser.parse_args()

class ServerNode(Node):
    port_param_name = 'rtsp_server_port'
    input_selector_param_name = 'selected_input'
    bitrate_pararm_name = 'bitrate'
    def __init__(self, node_name='rtsp_server_node'):
        super().__init__(node_name)

        self.server = GstServer()
        self.timer = self.create_timer(2, self.timer_callback)

        self.declare_parameter(self.port_param_name, str(opt.port), ParameterDescriptor(description='Port the RTSP service is attached to'))
        self.declare_parameter(self.input_selector_param_name, 0, ParameterDescriptor(description='Selected Input sink'))
        self.declare_parameter(self.bitrate_pararm_name, 50_000, ParameterDescriptor(description='Encoder Bitrate'))
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        print('node spun')

    def parameter_callback(self, params):
        success = False
        for param in params:
            if param.name == self.port_param_name:
                print('Cannot change port in flight')
            if param.name == self.input_selector_param_name:
                self.server.factory.switch_to_input(param.value)
                success = True
            if param.name == self.bitrate_pararm_name:
                self.server.factory.change_bitrate(param.value)
                success=True
            else:
                pass
        return SetParametersResult(successful=success)

import signal
def main():
    def handler(a,b):
        print('caught sigint')
        loop.quit()
        thread.join(0.5)

    # initializing the threads and running the stream on loop.
    signal.signal(signal.SIGINT, handler)
    Gst.init(None)
    rclpy.init()
    node = ServerNode()
    thread = Thread(target=rclpy.spin, args=[node])
    thread.start()
    loop = GLib.MainLoop.new(None, False)
    try:
        loop.run()
    except KeyboardInterrupt:
        thread.join(timeout=1)
        loop.quit()
    rclpy.shutdown()
    
if __name__ == '__main__':
  main()