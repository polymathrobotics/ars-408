#!/usr/bin/bash
set -e

# setup ros environment
source /opt/polymathrobotics/setup.bash

exec "$@"
