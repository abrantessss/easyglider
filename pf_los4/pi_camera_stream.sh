#!/usr/bin/env bash
set -euo pipefail
port="${1:-8888}"
device="${2:-/dev/video0}"
if [[ ! "$port" =~ ^[0-9]{1,5}$ ]] || ((10#$port < 1 || 10#$port > 65535)); then
    echo 'Camera port must be between 1 and 65535.' >&2
    exit 1
fi
port=$((10#$port))
# Prefer Raspberry Pi camera applications where available.
if command -v rpicam-vid >/dev/null 2>&1; then
    camera_command=rpicam-vid
elif command -v libcamera-vid >/dev/null 2>&1; then
    camera_command=libcamera-vid
else
    # Ubuntu 22.04 legacy CSI driver: bcm2835-v4l2 exposes H.264 on /dev/video0.
    # V4L2 here is the Pi's CSI camera interface, not USB camera capture.
    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo 'On the camera Pi, install FFmpeg: sudo apt install ffmpeg' >&2
        exit 1
    fi
    if [[ ! -c "$device" ]]; then
        echo "Camera device not found: $device. Check v4l2-ctl --list-devices." >&2
        exit 1
    fi
    echo "Streaming CSI camera $device on TCP port $port using FFmpeg (H.264 passthrough)." >&2
    exec ffmpeg -hide_banner -loglevel warning \
        -f v4l2 -input_format h264 -video_size 640x480 -framerate 15 -i "$device" \
        -an -c:v copy -f h264 "tcp://0.0.0.0:$port?listen=1"
fi
# Run on the Pi Zero 2 W with its CSI camera attached.
exec "$camera_command" -t 0 -n --codec h264 --inline --width 640 --height 480 \
    --framerate 15 --bitrate 1000000 --listen -o "tcp://0.0.0.0:$port"
