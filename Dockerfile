FROM koide3/gtsam_docker:humble AS main

RUN apt-fast update \
  && apt-fast install -y --no-install-recommends \
   ros-humble-rosbag2-storage-mcap libomp-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
   xvfb mesa-utils libgl1-mesa-glx libglu1-mesa libgl1-mesa-dri mesa-common-dev libglfw3-dev libglew-dev \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver \
  && cd ceres-solver \
  && git checkout e47a42c2957951c9fafcca9995d9927e15557069 \
  && mkdir build \
  && cd build \
  && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
  && make -j$(nproc) \
  && make install \
  && rm -rf /root/ceres-solver

COPY . /root/ros2_ws/src/direct_visual_lidar_calibration

WORKDIR /root/ros2_ws/src/direct_visual_lidar_calibration
RUN git submodule update --init --recursive

WORKDIR /root/ros2_ws

RUN apt-fast update && \
  rosdep install -i --from-paths src --simulate | \
  sed '1d' | sed 's/apt-get install//' | sed 's/ //g' > /tmp/depends && \
  xargs apt-fast install --no-install-recommends -y < /tmp/depends && \
  apt-fast clean && \
  rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/humble/setup.bash; colcon build"

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "export DISPLAY=:99" >> /ros_entrypoint.sh \
  && echo "Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &" >> /ros_entrypoint.sh \
  && echo "sleep 1" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/humble/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/ros2_ws/src/direct_visual_lidar_calibration

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


FROM main AS superglued 

RUN apt-fast update \
  && apt-fast install -y --no-install-recommends \
  python3-pip python3-numpy python3-torch python3-torchvision python3-matplotlib python3-opencv \
  && apt-fast clean \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/magicleap/SuperGluePretrainedNetwork /root/SuperGlue
ENV PYTHONPATH=$PYTHONPATH:/root/SuperGlue


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
