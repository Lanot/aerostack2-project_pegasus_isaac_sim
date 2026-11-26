#!/usr/bin/env python3
"""Relay an Isaac Sim ROS image topic to QGC using H.264/RTP over UDP."""
import argparse
import time

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


def image_bytes(message):
    """Pack 8-bit ROS images into GStreamer's four-byte-aligned row layout."""
    formats = {'rgb8': ('RGB', 3), 'bgr8': ('BGR', 3),
               'rgba8': ('RGBA', 4), 'bgra8': ('BGRA', 4), 'mono8': ('GRAY8', 1)}

    if message.encoding not in formats:
        raise ValueError(f'Unsupported encoding {message.encoding}; select a color image topic')

    format_name, channels = formats[message.encoding]
    row_bytes = message.width * channels

    if min(message.width, message.height) <= 0 or message.step < row_bytes:
        raise ValueError('Invalid image dimensions or row stride')

    data = bytes(message.data)
    if len(data) < message.step * message.height:
        raise ValueError('Image data is shorter than its declared dimensions')

    padding = bytes((-row_bytes) % 4)
    if message.step == row_bytes and not padding:
        data = data[:message.height * row_bytes]
    else:
        data = b''.join(data[row * message.step:row * message.step + row_bytes] + padding
                        for row in range(message.height))
    return format_name, data


def make_pipeline(width, height, fps, bitrate):
    """Use bounded queues so a slow encoder drops frames instead of adding latency."""
    pipeline = Gst.parse_launch(
        'appsrc name=frames is-live=true format=time do-timestamp=true min-latency=0 '
        'block=false max-buffers=1 leaky-type=downstream ! '
        'queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
        f'videoconvert ! videoscale ! video/x-raw,format=I420,width={width},height={height} ! '
        f'x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} '
        f'key-int-max={fps} bframes=0 byte-stream=true ! '
        'h264parse ! rtph264pay name=payloader pt=96 config-interval=1 mtu=1200 ! '
        'udpsink name=destination sync=false async=false')
    return pipeline


class VideoRelay(Node):
    def __init__(self, args):
        super().__init__('isaacsim_video_udp')

        self.args = args
        self.caps_signature = None
        self.last_frame = float('-inf')
        self.first_frame = True
        self.pipeline = make_pipeline(args.width, args.height, args.fps, args.bitrate)

        destination = self.pipeline.get_by_name('destination')
        destination.set_property('host', args.host)
        destination.set_property('port', args.port)

        self.source = self.pipeline.get_by_name('frames')
        self.bus = self.pipeline.get_bus()

        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError('GStreamer could not start')
        self.subscription = self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
        self.bus_timer = self.create_timer(0.5, self.check_pipeline)
        self.get_logger().warning(f'Video relay: {args.topic} -> udp://{args.host}:{args.port}')
        self.get_logger().info('Waiting for camera frames')

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_frame < 1.0 / self.args.fps:
            return
        try:
            format_name, data = image_bytes(message)
        except Exception as error:
            self.get_logger().error(f'Cannot convert camera image: {error}', throttle_duration_sec=5.0)
            return

        signature = (format_name, message.width, message.height)
        if signature != self.caps_signature:
            self.source.set_property('caps', Gst.Caps.from_string(
                f'video/x-raw,format={format_name},width={message.width},'
                f'height={message.height},framerate={self.args.fps}/1'))
            self.caps_signature = signature

        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        buffer.duration = Gst.SECOND // self.args.fps

        if self.source.emit('push-buffer', buffer) != Gst.FlowReturn.OK:
            raise RuntimeError('GStreamer stopped accepting frames')

        self.last_frame = now
        if self.first_frame:
            self.get_logger().info('First camera frame sent')
            self.first_frame = False

    def check_pipeline(self):
        message = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if message is None:
            return
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            raise RuntimeError(f'GStreamer: {error.message}; {debug}')
        raise RuntimeError('Video pipeline ended')

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/drone0/camera/color/image_raw')
    parser.add_argument('--host', default='127.0.0.1', help='IP address of the QGC computer')
    parser.add_argument('--port', type=int, default=5600)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=10, help='Maximum wall-clock send rate')
    parser.add_argument('--bitrate', type=int, default=1500, help='H.264 bitrate in kbit/s')
    args = parser.parse_args(remove_ros_args()[1:])

    if not 1 <= args.port <= 65535 or min(args.width, args.height, args.fps, args.bitrate) <= 0:
        parser.error('port must be 1..65535; dimensions, fps and bitrate must be positive')
    if args.width % 2 or args.height % 2:
        parser.error('width and height must be even')

    Gst.init(None)
    rclpy.init()
    node = None

    try:
        node = VideoRelay(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
