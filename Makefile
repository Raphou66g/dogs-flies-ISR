.PHONY: clean build src firmware

SHELL := /bin/bash

src:
	source install/setup.sh

clean:
	rm -rf ./build/ ./install/ ./log/

build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

firmware:
	export PYTHONPATH=./utils/crazyflie-firmware/build:$PYTHONPATH


rebuild: clean build src