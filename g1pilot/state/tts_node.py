#!/usr/bin/env python3
"""ROS 2 node that speaks text published on a topic through the G1 speaker.

Subscribes to /g1pilot/say (std_msgs/String) and plays each message via
espeak-ng + the Unitree AudioClient. A single worker thread drains a FIFO
queue, so the executor is never blocked and overlapping messages are spoken
one after another in arrival order.

Run:
    ros2 run g1pilot tts_node --ros-args -p interface:=eno2
"""

import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from g1pilot.state.say import synthesize_pcm, play_stream, play_stop, CHUNK_SIZE

# Robot startup latency before audio leaves the speaker; without waiting this
# out before PlayStop the tail of the utterance is truncated.
STARTUP_LATENCY = 0.5


class TtsNode(Node):
    def __init__(self):
        super().__init__("g1_tts_node")

        self.declare_parameter("interface", "")
        self.declare_parameter("topic", "/g1pilot/say")
        self.interface = str(self.get_parameter("interface").value)
        topic = str(self.get_parameter("topic").value)

        ChannelFactoryInitialize(0, self.interface)
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()

        # FIFO of pending utterances drained by a single worker thread, so
        # overlapping requests queue up and play in arrival order.
        self._queue = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.subscription = self.create_subscription(String, topic, self.say_callback, 10)
        self.get_logger().info(f"TTS node ready: speaking text from '{topic}'")

    def say_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self._queue.put(text)

    def _worker_loop(self):
        while True:
            text = self._queue.get()
            try:
                self._speak(text)
            except Exception as e:
                self.get_logger().error(f"[TTS] failed: {e}")
            finally:
                self._queue.task_done()

    def _speak(self, text):
        pcm = synthesize_pcm(text)
        if not pcm:
            self.get_logger().warn(f"No audio generated for: {text!r}")
            return
        audio_seconds = len(pcm) / 32000.0
        stream_id = str(int(time.time() * 1000))
        start = time.monotonic()
        for offset in range(0, len(pcm), CHUNK_SIZE):
            play_stream(self.audio_client, stream_id, pcm[offset:offset + CHUNK_SIZE])
            time.sleep(1)
        remaining = audio_seconds + STARTUP_LATENCY - (time.monotonic() - start)
        if remaining > 0:
            time.sleep(remaining)
        play_stop(self.audio_client, stream_id)


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
