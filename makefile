PROGRAM = nvidia-docker
# XSOCK=/tmp/.X11-unix
# XAUTH=/tmp/.docker.xauth
HARBOR_REPO = harbor.pfn.io/hayate/hayate_gpd:latest

build:
	$(PROGRAM) build -t $(HARBOR_REPO) . 

pull:
	docker pull $(HARBOR_REPO)
push:
	docker push $(HARBOR_REPO)
run:
	# chmod +r ~/.Xauthority
	# $(PROGRAM) run -v -it --security-opt label=disable --security-opt seccomp=unconfined --privileged --device=/dev/dri:/dev/dri --net host --env QT_X11_NO_MITSHM=1 \gpd_8.0:latest /bin/bash
	$(PROGRAM) run -it --privileged --net host -v /home/kusano/hayate_gpd/annotation:/catkin_ws/src/gpd/annotation --env-file ./env.list $(HARBOR_REPO)

killall:
	$(PROGRAM) rm `$(PROGRAM) ps -aq`

prune:
	$(PROGRAM) system prune

