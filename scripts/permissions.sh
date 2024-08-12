#!/usr/bin/env bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)

if [ ! -f "/etc/udev/rules.d/eys3d_depth-8062.rules" ]; then
    echo "config device permission"
    sudo cp $BASE_DIR/config/eys3d_depth-8062.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo service udev restart
    sudo udevadm trigger
    echo "... done"
fi
