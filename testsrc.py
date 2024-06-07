import argparse
import gi
# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib

class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.launch_string = 'filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! rtph265pay name="pay0"'
        #Share pipeline between clients
        self.set_shared(True)
        self.own_media = None

    # attach the launch string to the override method
    def do_create_element(self, url):
        elem = Gst.parse_launch(self.launch_string)
        print("start stream")
        return elem
    
    # set configurable properties on the constructed media object
    def do_configure(self, rtsp_media):
        self.media = rtsp_media
        self.number_frames = 0
        # rtsp_media.set_shared(True)
        # rtsp_media.set_reusable(True)

class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.set_service(str(opt.port))

        self.factory1 = SensorFactory()
        self.factory1.set_shared(True)
        self.factory1.latency = 100
        self.get_mount_points().add_factory(opt.stream_uri+"1", self.factory1)

        self.factory2 = SensorFactory()
        self.factory2.launch_string = 'filesrc location="/home/peter/Videos/sintel-4096x1744-cfg02.mkv" ! matroskademux ! h265parse ! rtph265pay name="pay0"'
        self.factory2.set_shared(True)
        self.factory2.latency = 100
        self.get_mount_points().add_factory(opt.stream_uri+"2", self.factory2)

        self.attach(None)
        print('Server attached')

# getting the required information from the user 
parser = argparse.ArgumentParser()
parser.add_argument("--port", default=8554, help="port to stream video", type = int)
parser.add_argument("--stream_uri", default = "/video_stream", help="rtsp video stream uri")
opt = parser.parse_args()

def main():
    Gst.init(None)
    server = GstServer()
    loop = GLib.MainLoop.new(None, False)
    loop.run()
    
if __name__ == '__main__':
  main()