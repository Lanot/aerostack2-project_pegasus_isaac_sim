"""Optional relay tests; requires ROS Jazzy and GStreamer on the test host."""
import importlib.util
from pathlib import Path
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('stream_video_udp', ROOT / 'scripts/stream_video_udp.py')
relay = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(relay)
except (ImportError, ValueError) as error:
    raise unittest.SkipTest(f'ROS/GStreamer relay dependencies unavailable: {error}')


class VideoTests(unittest.TestCase):
    def test_rows_are_aligned_without_corrupting_pixels(self):
        image = types.SimpleNamespace(encoding='rgb8', width=1, height=2, step=4,
                                     data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
        format_name, data = relay.image_bytes(image)
        self.assertEqual(format_name, 'RGB')
        self.assertEqual(data, bytes([1, 2, 3, 0, 4, 5, 6, 0]))
        image.encoding = '32FC1'
        with self.assertRaises(ValueError):
            relay.image_bytes(image)
        image.encoding = 'rgb8'
        image.data = b'123'
        with self.assertRaises(ValueError):
            relay.image_bytes(image)

    def test_pipeline_produces_h264_rtp_payload_type_96(self):
        Gst = relay.Gst
        Gst.init(None)
        if not all(Gst.ElementFactory.find(name) for name in ('x264enc', 'h264parse', 'rtph264pay')):
            self.skipTest('H.264/RTP GStreamer plugins unavailable')
        pipeline = relay.make_pipeline(160, 90, 10, 500)
        # Capture actual RTP packets in memory without a network/GPU dependency.
        destination = pipeline.get_by_name('destination')
        payloader = pipeline.get_by_name('payloader')
        payloader.unlink(destination)
        pipeline.remove(destination)
        sink = Gst.ElementFactory.make('appsink')
        sink.set_property('sync', False)
        pipeline.add(sink)
        self.assertTrue(payloader.link(sink))
        source = pipeline.get_by_name('frames')
        source.set_property('caps', Gst.Caps.from_string(
            'video/x-raw,format=RGB,width=4,height=2,framerate=10/1'))
        try:
            pipeline.set_state(Gst.State.PLAYING)
            data = bytes([255, 0, 0] * 8)
            frame = Gst.Buffer.new_allocate(None, len(data), None)
            frame.fill(0, data)
            frame.duration = Gst.SECOND // 10
            self.assertEqual(source.emit('push-buffer', frame), Gst.FlowReturn.OK)
            sample = sink.emit('try-pull-sample', 5 * Gst.SECOND)
            self.assertIsNotNone(sample)
            header = sample.get_buffer().extract_dup(0, 2)
            self.assertEqual(header[0] >> 6, 2)
            self.assertEqual(header[1] & 0x7f, 96)
        finally:
            pipeline.set_state(Gst.State.NULL)
