ifeq ($(OS),Windows_NT) 
	detected_OS := Windows
else
	detected_OS := $(shell sh -c 'uname 2>/dev/null || echo Unknown')
endif

define runMap
  sudo apt install x11-xserver-utils; \
  xhost +local:docker; \
  sudo docker run -it --net=host -v $(dir $(realpath $(lastword $(MAKEFILE_LIST)))):/code -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$$DISPLAY cttewari/gtsam_dev:latest
endef

init:
	pip3 install -r requirements.txt

dockerpull:
	docker pull cttewari/gtsam_dev

dockerbuild:
	docker build -t cttewari/gtsam_dev .
	
dockerrun:
ifeq ($(detected_OS),Linux)
ifeq ($(origin WSLENV),undefined)
	export DISPLAY=:0; \
	$(call runMap)
else
	export DISPLAY=`grep -oP "(?<=nameserver ).+" /etc/resolv.conf`:0.0; \
	$(call runMap)
endif
else
	docker run -it --net=host -v $(dir $(realpath $(lastword $(MAKEFILE_LIST)))):/code cttewari/gtsam_dev:latest
endif
	
formatpython:
	black -l 79 client
	black -l 79 client/tests
