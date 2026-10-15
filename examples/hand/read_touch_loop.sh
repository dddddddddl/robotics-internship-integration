#!/usr/bin/env bash
# Read-only B-hand acquisition; source ROS 2 and the built workspace first.
command -v ros2 >/dev/null || { echo "Source ROS 2 and the robot workspace first." >&2; exit 1; }
command -v timeout >/dev/null || { echo "GNU timeout is required (Ubuntu: coreutils)." >&2; exit 1; }
trap 'echo; echo "Hand monitor stopped"; exit 0' INT TERM
echo "Reading B arm / COM3 / hand ID1. Press Ctrl+C to stop."
while true; do
    date -u '+%Y-%m-%dT%H:%M:%SZ'
    timeout 7s ros2 service call /hand/read_touch \
        inspire_hand_interface/srv/ReadTouch \
        "{arm_id: 'B', com: 3, hand_id: 1, timeout_sec: 5.0}"
    result=$?
    if [ "$result" -eq 124 ]; then
        echo "Read timed out; continuing."
    elif [ "$result" -ne 0 ]; then
        echo "Service call failed with status $result."
    fi
    sleep 0.5
done
