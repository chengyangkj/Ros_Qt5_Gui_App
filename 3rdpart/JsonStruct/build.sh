#!/bin/sh -vx
set -e

if [ -e "build" ]; then
    rm -rf "build"
fi

mkdir build
cd build

if [[ "$OS" == "Windows_NT" ]]; then
    cmake ..
    cmake --build . --config Release
    ctest -C Release --output-on-failure .
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .
    ctest -C Release --output-on-failure .
fi
