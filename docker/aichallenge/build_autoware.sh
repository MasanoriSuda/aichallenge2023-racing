#!/bin/bash

cd ./aichallenge_ws
pip install setuptools==58.2.0
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
