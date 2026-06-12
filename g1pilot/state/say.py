"""Speak English text on the G1 speaker via Piper (neural TTS) + PlayStream.

Synthesis backend: Piper if installed (natural voice), else espeak-ng
(robotic fallback). Requires `ffmpeg` for resampling:
    apt-get install -y ffmpeg
Piper binary + voice model (override paths with PIPER_BIN / PIPER_MODEL):
    /opt/piper/piper  +  /opt/piper-voices/en_US-lessac-medium.onnx

Pauses: messages may embed <break time="500ms"/> or <break time="2s"/> tags;
these are rendered as real spliced silence (works with either backend).

Usage:
    python3 say.py <network_interface> <text...>
"""

import json
import os
import re
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

SAMPLE_RATE = 16000  # G1 speaker expects 16kHz mono 16-bit PCM
# Trailing silence appended to every utterance. The robot has a variable
# startup latency before audio reaches the speaker; without padding, PlayStop
# can fire before the last words play out and clip the tail. This pad absorbs
# that jitter so only silence is ever clipped. Override with SAY_TAIL_SILENCE.
TRAILING_SILENCE_S = float(os.environ.get("SAY_TAIL_SILENCE", "1.0"))

# Piper (neural TTS). Override with env vars to swap the voice/model.
PIPER_BIN = os.environ.get("PIPER_BIN", "/opt/piper/piper")
PIPER_MODEL = os.environ.get("PIPER_MODEL", "/opt/piper-voices/en_US-lessac-medium.onnx")
# Phoneme length scale: >1.0 slower, <1.0 faster (1.0 = model default).
PIPER_LENGTH_SCALE = os.environ.get("PIPER_LENGTH_SCALE", "1.0")

# <break time="500ms"/> or <break time="2s"/> -> spliced silence (engine-agnostic).
_BREAK_RE = re.compile(r'<break\s+time="(\d+(?:\.\d+)?)(ms|s)"\s*/>')


def _piper_available() -> bool:
    return os.path.exists(PIPER_BIN) and os.path.exists(PIPER_MODEL)


def _to_pcm(audio_bytes: bytes) -> bytes:
    """Resample arbitrary audio (WAV/etc.) to 16kHz mono 16-bit PCM via ffmpeg."""
    return subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-i", "pipe:0",
         "-ar", str(SAMPLE_RATE), "-ac", "1", "-f", "s16le", "pipe:1"],
        input=audio_bytes, capture_output=True, check=True,
    ).stdout


def _silence_pcm(seconds: float) -> bytes:
    return b"\x00\x00" * int(seconds * SAMPLE_RATE)


def _synth_segment(text: str) -> bytes:
    """Synthesize one (tag-free for Piper) text segment to 16kHz mono PCM."""
    if _piper_available():
        wav = subprocess.run(
            [PIPER_BIN, "-m", PIPER_MODEL, "-q",
             "--length_scale", PIPER_LENGTH_SCALE, "-f", "-"],
            input=text.encode(), capture_output=True, check=True,
        ).stdout
        return _to_pcm(wav)
    # Fallback: espeak-ng (robotic). -m so it renders any <break> tags itself.
    wav = subprocess.run(
        ["espeak-ng", "-m", "-v", "en", "-s", "160", "--stdout", text],
        capture_output=True, check=True,
    ).stdout
    return _to_pcm(wav)


def synthesize_pcm(text: str) -> bytes:
    """Convert text to 16kHz mono 16-bit PCM for the G1 speaker.

    Supports <break time="Ns"/> / <break time="Nms"/> tags. With Piper these are
    spliced in as real silence; the espeak-ng fallback renders them natively via
    SSML, so the whole string is passed through untouched in that case.

    A trailing silence pad is appended so PlayStop never clips real words."""
    pad = _silence_pcm(TRAILING_SILENCE_S)
    if not _piper_available():
        return _synth_segment(text) + pad
    parts = []
    last = 0
    for m in _BREAK_RE.finditer(text):
        seg = text[last:m.start()].strip()
        if seg:
            parts.append(_synth_segment(seg))
        val = float(m.group(1))
        parts.append(_silence_pcm(val / 1000.0 if m.group(2) == "ms" else val))
        last = m.end()
    tail = text[last:].strip()
    if tail:
        parts.append(_synth_segment(tail))
    parts.append(pad)
    return b"".join(parts)


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
