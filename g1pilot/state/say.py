"""Speak English text on the G1 speaker via espeak-ng + PlayStream.

Requires `espeak-ng` and `ffmpeg` in the container:
    apt-get install -y espeak-ng ffmpeg

Usage:
    python3 say.py <network_interface> <text...>
"""

import json
import subprocess
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
    Request_ as Request,
    RequestHeader_ as RequestHeader,
    RequestLease_ as RequestLease,
    RequestIdentity_ as RequestIdentity,
    RequestPolicy_ as RequestPolicy,
)
from unitree_sdk2py.utils.future import FutureResult

API_ID_START_PLAY = 1003
API_ID_STOP_PLAY = 1004
CHUNK_SIZE = 96000  # matches Unitree's C++ example: 3s of 16kHz mono 16-bit PCM
APP_NAME = "g1pilot"


def synthesize_pcm(text: str) -> bytes:
    espeak = subprocess.Popen(
        ["espeak-ng", "-v", "en", "-s", "160", "--stdout", text],
        stdout=subprocess.PIPE,
    )
    ffmpeg = subprocess.run(
        [
            "ffmpeg", "-loglevel", "error",
            "-i", "pipe:0",
            "-ar", "16000", "-ac", "1", "-f", "s16le",
            "pipe:1",
        ],
        stdin=espeak.stdout,
        capture_output=True,
        check=True,
    )
    espeak.wait()
    return ffmpeg.stdout


# PlayStream needs both a JSON parameter AND a binary payload in the same
# request, but the Python SDK's _Call sends only JSON and _CallBinary sends
# only bytes. Build the Request manually to carry both.
def _call_param_and_binary(client, api_id, parameter, binary):
    stub = client._ClientBase__stub
    timeout = client._ClientBase__timeout
    header = RequestHeader(
        RequestIdentity(time.monotonic_ns(), api_id),
        RequestLease(0),
        RequestPolicy(0, False),
    )
    request = Request(header, parameter, list(binary))
    future = stub.SendRequest(request, timeout)
    if future is None:
        return -1
    result = future.GetResult(timeout)
    if result.code != FutureResult.FUTURE_SUCC:
        stub.RemoveFuture(request.header.identity.id)
        return -2
    return result.value.header.status.code


def play_stream(client, stream_id, pcm):
    return _call_param_and_binary(
        client,
        API_ID_START_PLAY,
        json.dumps({"app_name": APP_NAME, "stream_id": stream_id}),
        pcm,
    )


def play_stop(client, stream_id):
    code, _ = client._Call(
        API_ID_STOP_PLAY, json.dumps({"app_name": stream_id})
    )
    return code


def main():
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} <iface> <text...>")
        sys.exit(1)
    iface = sys.argv[1]
    text = " ".join(sys.argv[2:])

    ChannelFactoryInitialize(0, iface)
    client = AudioClient()
    client.SetTimeout(10.0)
    client.Init()

    pcm = synthesize_pcm(text)
    if not pcm:
        print("no audio generated", file=sys.stderr)
        sys.exit(2)

    audio_seconds = len(pcm) / 32000.0

    # The robot has a startup latency before audio leaves the speaker; wait
    # for that + the audio duration before PlayStop, or it truncates playback.
    STARTUP_LATENCY = 0.5

    stream_id = str(int(time.time() * 1000))
    start = time.monotonic()
    for offset in range(0, len(pcm), CHUNK_SIZE):
        play_stream(client, stream_id, pcm[offset:offset + CHUNK_SIZE])
        time.sleep(1)
    remaining = audio_seconds + STARTUP_LATENCY - (time.monotonic() - start)
    if remaining > 0:
        time.sleep(remaining)
    play_stop(client, stream_id)


if __name__ == "__main__":
    main()
