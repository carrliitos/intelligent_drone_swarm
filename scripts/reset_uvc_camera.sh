#!/bin/bash

DEV=/dev/video2

# Restore auto exposure (Aperture Priority mode)
v4l2-ctl -d $DEV --set-ctrl=auto_exposure=3

# Restore gain default
v4l2-ctl -d $DEV --set-ctrl=gain=64

# Restore exposure dynamic framerate default
v4l2-ctl -d $DEV --set-ctrl=exposure_dynamic_framerate=0

# Restore basic image controls
v4l2-ctl -d $DEV --set-ctrl=brightness=128,contrast=32,saturation=32,sharpness=24,backlight_compensation=0,power_line_frequency=2

# Restore white balance auto
v4l2-ctl -d $DEV --set-ctrl=white_balance_automatic=1

# Set back to 1280x720 MJPG at 30fps (typical UVC Camera default)
v4l2-ctl -d $DEV --set-fmt-video=width=1280,height=720,pixelformat=MJPG
v4l2-ctl -d $DEV --set-parm=30

# Confirm new settings
v4l2-ctl -d $DEV --all
