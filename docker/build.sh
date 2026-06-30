#!/bin/bash
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Clone forest workspace sources if not already present
if [ ! -d "$SCRIPT_DIR/src" ] || [ -z "$(ls -A "$SCRIPT_DIR/src" 2>/dev/null)" ]; then
    echo "Cloning forest workspace sources..."
    "$SCRIPT_DIR/clone_src.sh"
fi

docker build -t g1pilot .