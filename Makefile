.PHONY: clean build src

SHELL := /bin/bash

src:
	source install/setup.sh

clean:
	rm -rf ./build/ ./install/ ./log/

build:
	colcon build --symlink-install

rebuild: clean build src