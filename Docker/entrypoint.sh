#!/bin/bash

source /opt/ros/humble/setup.sh
source /opt/underlay_ws/install/setup.bash
source /opt/overlay_ws/install/setup.bash
exec "$@"