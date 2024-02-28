# GUI_2024

# WASSSSUP

# For å starte videopipeline:

# Avsender Pc: gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=RECEIVER_IP port=5000

# Mottaker pc: gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp, payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
