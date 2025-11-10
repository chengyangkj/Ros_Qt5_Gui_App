#!/bin/bash
cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$(dirname "$0")/lib:$LD_LIBRARY_PATH"
./ros_qt5_gui_app