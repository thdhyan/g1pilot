# Bakes the g1pilot speech stack into the image so g1pilot.state.say works with
# no runtime apt install:
#   - Piper (neural TTS) binary + a natural English voice model  -> /opt/piper*
#   - espeak-ng (robotic fallback synth) + ffmpeg (resample to 16 kHz)
#
# Build (from the docker/ dir):
#   docker build -f piper.Dockerfile \
#     -t registry.gitlab.com/bleurobotics/code/bleu_integration/g1pilot-from-bleu-base:2026.06 .
#
# Override the base or voice at build time with --build-arg if needed.
ARG BASE=registry.gitlab.com/bleurobotics/code/bleu_integration/g1pilot-from-bleu-base:2026.03
FROM ${BASE}

ARG PIPER_VERSION=2023.11.14-2
ARG PIPER_VOICE=en_US-lessac-medium
ARG PIPER_VOICE_URL=https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium

# Fallback synth + resampler (matches what the old entrypoint installed at runtime).
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        espeak-ng ffmpeg curl ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Piper neural TTS binary (bundles its own libs + espeak-ng-data under /opt/piper).
RUN curl -fsSL -o /tmp/piper.tar.gz \
        https://github.com/rhasspy/piper/releases/download/${PIPER_VERSION}/piper_linux_x86_64.tar.gz \
 && tar -xzf /tmp/piper.tar.gz -C /opt \
 && rm /tmp/piper.tar.gz

# Voice model.
RUN mkdir -p /opt/piper-voices \
 && curl -fsSL -o /opt/piper-voices/${PIPER_VOICE}.onnx      ${PIPER_VOICE_URL}/${PIPER_VOICE}.onnx \
 && curl -fsSL -o /opt/piper-voices/${PIPER_VOICE}.onnx.json ${PIPER_VOICE_URL}/${PIPER_VOICE}.onnx.json

# say.py reads these (defaults already match; set explicitly so the voice/speed
# can be overridden per-container without code changes).
ENV PIPER_BIN=/opt/piper/piper \
    PIPER_MODEL=/opt/piper-voices/${PIPER_VOICE}.onnx \
    PIPER_LENGTH_SCALE=1.0

# Sanity-check the binary is runnable at build time (fails the build otherwise).
RUN /opt/piper/piper --help >/dev/null
