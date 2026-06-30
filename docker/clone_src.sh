#!/bin/bash
# Clone forest workspace source packages into docker/src/

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

echo "Cloning forest workspace packages into $SRC_DIR..."

# Key packages for forest builds (used in Dockerfile COPY steps)
repos=(
    "https://github.com/coal-library/coal.git:coal"
    "https://github.com/stack-of-tasks/pinocchio.git:pinocchio"
    "https://github.com/ADVRHumanoids/xbot2_interface.git:xbot2_interface"
    "https://github.com/oxfordcontrol/osqp.git:osqp"
    "https://github.com/Simple-Robotics/proxsuite.git:proxsuite"
    "https://github.com/flexible-collision-library/fcl.git:fcl"
    "https://github.com/qpSWIFT/qpSWIFT.git:qpSWIFT"
)

for repo in "${repos[@]}"; do
    url="${repo%%:*}"
    dirname="${repo##*:}"

    if [ -d "$dirname" ]; then
        echo "✓ $dirname already exists, skipping..."
    else
        echo "⬇ Cloning $dirname from $url..."
        git clone "$url" "$dirname"
    fi
done

echo "✓ Done. All source packages cloned to $SRC_DIR/"
