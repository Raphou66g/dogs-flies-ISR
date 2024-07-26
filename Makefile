.PHONY: clean build src firmware

SHELL := /bin/bash

src:
	source install/setup.sh

clean:
	rm -rf ./build/ ./install/ ./log/

firmware:
	export PYTHONPATH=~/crazyflie-firmware/build:$PYTHONPATH

build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
	export PYTHONPATH=~/crazyflie-firmware/build:$PYTHONPATH


rebuild: clean build src