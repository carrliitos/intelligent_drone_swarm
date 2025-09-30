#!/bin/bash
timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
outfile="camera_settings_${timestamp}.txt"

v4l2-ctl -d /dev/video2 --all > "$outfile"

echo "Saved camera settings to $outfile"
