#!/bin/bash

xhost +

DIR=$(pwd)

# Determine the WSL host IP address dynamically
# export WSL_HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')

#sudo docker rm $1

docker run -d --name $1 -w /home/rosdev/social_gym/   --privileged \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=host.docker.internal:0.0 \
-e LIBGL_ALWAYS_SOFTWARE=1 \
-e QT_X11_NO_MITSHM=1 \
-v ${DIR}/data:/home/rosdev/social_gym/data \
-v ${DIR}/config_runner/configs:/home/rosdev/social_gym/config_runner/configs \
-v ${DIR}/submodules:/home/rosdev/social_gym/submodules \
-v ${DIR}/maps:/home/rosdev/social_gym/maps \
-v ${DIR}/manifest.xml:/home/rosdev/social_gym/manifest.xml \
-v ${DIR}/config/gym_gen:/home/rosdev/social_gym/config/gym_gen \
-v ${DIR}/config_runner/set_paths.sh:/home/rosdev/social_gym/config_runner/set_paths.sh \
--network host \
-v ${DIR}/src:/home/rosdev/social_gym/src \
social_gym_config_runner:1.0 \
bash -c \
"(source config_runner/set_paths.sh && roscore &) && sleep 4 && (export DOCKER=false && source config_runner/set_paths.sh && export PYTHONPATH=\$PYTHONPATH:/home/rosdev/social_gym && pip show supersuit && pip show pettingzoo && pip show stable-baselines3 && python --version && python -u src/config_run.py -c ${2})"

