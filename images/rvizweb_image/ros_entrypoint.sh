#!/bin/bash
set -e

# setup ros environment
source "/root/rvizweb_ws/install/setup.bash"
exec "$@"
