#!/usr/bin/env python3

import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gst, Gtk, GLib, Gdk


class Player(Gtk.Window):

    bitrate = 256

    def __init__(self):
        # initialize GTK
        super().__init__(title="Gstream viewer")

        # initialize GStreamer
        Gst.init(sys.argv)

        self.state = Gst.State.NULL
        self.duration = Gst.CLOCK_TIME_NONE
        # self.pipeline = Gst.parse_launch('uridecodebin name=source ! videoconvert ! gtksink name=output')
        self.pipeline = Gst.parse_launch(f'input-selector name="selector" ! capsfilter name="size" caps="video/x-raw, height=900, width=1100, framerate=30/1" !  timeoverlay  ! vaapipostproc ! \
                                          vaapih265enc name="encoder" bitrate={self.bitrate} rate-control="cbr" ! h265parse ! \
                                          vaapih265dec ! vaapisink name=output \
                                          videotestsrc pattern=21 kt=8 is-live=true ! selector.sink_0 \
                                          videotestsrc pattern=0 is-live=true ! selector.sink_1')
        # self.pipeline = Gst.parse_launch(f'input-selector name="selector" ! vaapipostproc name="size" ! \
        #                                  vaapih265enc name="encoder" bitrate={self.bitrate} rate-control="cbr" ! \
        #                                  vaapih265dec ! vaapisink name=output \
        #                                  filesrc location="/home/peter/Videos/sintel-4096x1744-cfg02.mkv" ! matroskademux ! h265parse ! vaapih265dec ! selector.sink_0 \
        #                                  filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse ! vaapih265dec ! selector.sink_1')
        # self.pipeline = Gst.parse_launch(f'input-selector name="selector" ! vaapih265dec ! vaapipostproc name="size" ! \
        #                                  vaapih265enc name="encoder" bitrate={self.bitrate} rate-control="cbr" ! \
        #                                  capsfilter name="size" caps="video/x-raw(memory:NVMM), width=(int)800, height=(int)800, format=(string)I420" ! \
        #                                  vaapih265dec ! vaapisink name=output \
        #                                  filesrc location="/home/peter/Videos/sintel-4096x1744-cfg02.mkv" ! matroskademux ! h265parse ! selector.sink_0 \
        #                                  filesrc location="/home/peter/Videos/bbb-3840x2160-cfg02.mkv" ! matroskademux ! h265parse !  selector.sink_1')
        # self.pipeline = Gst.parse_launch(f'input-selector name="selector" ! capsfilter caps="video/x-raw, height=600, width=800, framerate=30/1" ! \
        #                                   nvvidconv name=converter ! capsfilter name="size" caps="video/x-raw(memory:NVMM), width=(int)800, height=(int)800, format=(string)I420" !\
        #                                   nvv4l2h265enc name="encoder" control-rate=1 ratecontrol-enable=true bitrate={self.bitrate} ! h265parse ! \
        #                                   avdec_h265 ! videoconvert ! xvimagesink name=output \
        #                                   videotestsrc pattern=21 kt=8 is-live=true ! selector.sink_0 \
        #                                   videotestsrc pattern=0 is-live=true ! selector.sink_1')

        if not self.pipeline:
            print("ERROR: Could not create pipeline.")
            sys.exit(1)

        self.selector = self.pipeline.get_by_name('selector')
        if not self.selector:
            print("ERROR: Could not create selector.")
            sys.exit(1)
        self.encoder = self.pipeline.get_by_name('encoder')
        if not self.encoder:
            print("ERROR: Could not create encoder.")
            sys.exit(1)
        self.outsink = self.pipeline.get_by_name('output')
        if not self.outsink:
            print("ERROR: Could not create gtksink.")
            sys.exit(1)
        self.size = self.pipeline.get_by_name('size')
        if not self.size:
            print("ERROR: Could not create size.")
            sys.exit(1)

        # create the GUI
        self.build_ui()

        # instruct the bus to emit signals for each received message
        # and connect to the interesting signals
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_error)
        bus.connect("message::eos", self.on_eos)
        bus.connect("message::state-changed", self.on_state_changed)

    # set the pipeline to PLAYING (start playback)
    # and start the GTK main loop
    def start(self):
        # start playing
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("ERROR: Unable to set the pipeline to the playing state")
            sys.exit(1)

        # start the GTK main loop. we will not regain control until
        # Gtk.main_quit() is called
        self.show_all()
        Gtk.main()

        # free resources
        self.cleanup()

    def showBitrate(self):
        print(self.encoder.get_property('bitrate'))
    # set the pipeline state to NULL and remove the reference to it
    def cleanup(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

    def build_ui(self):
        self.connect("delete-event", self.on_delete_event)
        self.connect("key-press-event", self.process_input)

        grid = Gtk.Grid()
        increase_br_but = Gtk.Button(label='+')
        increase_br_but.connect('clicked', self.increase_bitrate)
        grid.attach(increase_br_but, left=2, top=0, width=1, height=1)
        decrease_br_but = Gtk.Button(label='-')
        decrease_br_but.connect('clicked', self.decrease_bitrate)
        grid.attach(decrease_br_but, left=0, top=0, width=1, height=1)
        switch_but = Gtk.Button(label='<->')
        switch_but.connect('clicked', self.switch_input)
        grid.attach(switch_but, left=1, top=1, width=1, height=1)

        self.add(grid)
        # self.add(video_window)
        # self.set_default_size(200, 200)
        self.set_resizable(False)
        self.set_position(Gtk.WindowPosition.CENTER)
        # self.show_all()
        # self.move(0,0)

    # this function is called when the main window is closed
    def on_delete_event(self, widget, event):
        Gtk.main_quit()

    # this function is called when an error message is posted on the bus
    def on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print("ERROR:", msg.src.get_name(), ":", err.message)
        if dbg:
            print("Debug info:", dbg)

    # this function is called when an End-Of-Stream message is posted on the bus
    # we just set the pipeline to READY (which stops playback)
    def on_eos(self, bus, msg):
        print("End-Of-Stream reached")
        self.pipeline.set_state(Gst.State.READY)

    # this function is called when the pipeline changes states.
    # we use it to keep track of the current state
    def on_state_changed(self, bus, msg):
        old, new, pending = msg.parse_state_changed()
        if not msg.src == self.pipeline:
            # not from the pipeline, ignore
            return

        self.state = new
        print("State changed from {0} to {1}".format(
            Gst.Element.state_get_name(old), Gst.Element.state_get_name(new)))

    def switch_input(self, button):
        active_pad = self.selector.get_property('active-pad')
        active_pad_name = active_pad.get_name()
        print(active_pad_name)
        if active_pad_name == 'sink_0':
            padname = 'sink_1'
        else:
            padname = 'sink_0'
        newpad = self.selector.get_static_pad(padname)
        self.selector.set_property('active-pad', newpad)

    def increase_bitrate(self, button):
        self.bitrate = self.bitrate * 2
        print(self.bitrate)
        self.encoder.set_property('bitrate', self.bitrate)
        print(self.encoder.get_property('bitrate'))
    def decrease_bitrate(self, button):
        self.bitrate = int(self.bitrate / 2)
        print(self.bitrate)
        self.encoder.set_property('bitrate', self.bitrate)
        print(self.encoder.get_property('bitrate'))

    def process_input(self, window, event):
        keyname = Gdk.keyval_name(event.keyval)
        # print(keyname)
        if keyname == 'Return':
            print('switching input')
            self.switch_input(None)
        elif keyname == 'KP_Add':
            print('increasing bitrate')
            self.increase_bitrate(None)
        elif keyname == 'KP_Subtract':
            print('reducing bitrate')
            self.decrease_bitrate(None)
        elif keyname in ['KP_2','KP_4','KP_6','KP_8']:
            caps = self.size.get_property('caps')
            print(caps.to_string())
            
            caps2 = Gst.Caps.from_string(caps.to_string())
            if keyname == 'KP_2':
                value, height = caps.get_structure(0).get_int('height')
                if value and height >= 100:
                    caps2.set_value('height', height-100)
                    self.size.set_property('caps', caps2)
            if keyname == 'KP_4':
                value, width = caps.get_structure(0).get_int('width')
                if value and width >= 100:
                    caps2.set_value('width', width-100)
                    self.size.set_property('caps', caps2)
            if keyname == 'KP_6':
                value, width = caps.get_structure(0).get_int('width')
                if value and width <= 1000:
                    caps2.set_value('width', width+100)
                    self.size.set_property('caps', caps2)
            if keyname == 'KP_8':
                value, height = caps.get_structure(0).get_int('height')
                if value and height <= 1000:
                    caps2.set_value('height', height+100)
                    self.size.set_property('caps', caps2)



if __name__ == '__main__':
    p = Player()
    p.start()
