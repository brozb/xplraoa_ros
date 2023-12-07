# ROS driver for u-blox XPLR-AOA1
Node `bt_angle.py` communicates with the anchor over UART and publishes angle measurements as a custom message [Angles](msg/Angles.msg).
The node publishes the raw angles received from the anchor, but also the average of last few measurements.

# Usage
A launch file for single anchor is provided ([anchor_node.launch](launch/anchor_node.launch)). The launch file
allows the user to specify the number of samples that are being averaged. It is possible to set ID for the Bluetooth anchor,
such that multiple anchors can be used at the same time. The measured angles are therefore published on topics `/bt_angle/ID/angles_raw` and `/bt_angle/ID/angles_avg`.
