#!/usr/bin/env bash
# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly)
if [ -z "$G1_INTERFACE" ]; then
    echo "ERROR: G1_INTERFACE environment variable is not set."
    echo "Set it to your network interface, e.g.: G1_INTERFACE=eno2"
    exit 1
fi

# Speech synthesis dependencies for g1pilot.state.say (espeak-ng + ffmpeg).
if ! command -v espeak-ng >/dev/null 2>&1 || ! command -v ffmpeg >/dev/null 2>&1; then
    DEBIAN_FRONTEND=noninteractive apt-get update \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends espeak-ng ffmpeg
fi

if [ "${SIMULATION,,}" = "true" ] || [ "$SIMULATION" = "1" ]; then
    # .bashrc (baked into the image) re-binds CYCLONEDDS_URI to G1_INTERFACE on every
    # interactive shell. In sim, override it back to the default-interface config that
    # docker-compose mounts at /cyclonedds.xml, so Cyclone picks any UP interface.
    SETUP_URI_CMD='export CYCLONEDDS_URI=file:///cyclonedds.xml && echo "SIMULATION=true -> CYCLONEDDS_URI=${CYCLONEDDS_URI} (default interface selection)"'
else
    SETUP_URI_CMD='source setup_uri.sh ${G1_INTERFACE}'
fi

exec bash -ic "
cd /ros2_ws &&
./cbuild &&
${SETUP_URI_CMD} &&
source install/setup.bash &&
ros2 launch g1pilot \${G1_LAUNCH_FILE:-bringup_opensot.launch.py} \
    enable_collision_avoidance:=\${ENABLE_COLLISION_AVOIDANCE:-false} \
    enable_external_collision_avoidance:=\${ENABLE_EXTERNAL_COLLISION_AVOIDANCE:-false} \
    box_pose_topic:=\${BOX_POSE_TOPIC:-/g1pilot/box_pose}
"
#unset RMW_IMPLEMENTATION &&
