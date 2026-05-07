#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ -f /workspace/install/setup.bash ]]; then
    set +u
    source /workspace/install/setup.bash
    set -u
fi

exec "$@"
