FROM public.ecr.aws/artefacts/ros2:humble-fortress

RUN wget https://mirror.clarkson.edu/blender/release/Blender3.4/blender-3.4.1-linux-x64.tar.xz && \
  tar -xvf blender-3.4.1-linux-x64.tar.xz --strip-components=1 -C /bin && \
  rm -rf blender-3.4.1-linux-x64.tar.xz && \
  rm -rf blender-3.4.1-linux-x64

COPY . /ws/src

WORKDIR /ws

RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential

WORKDIR /ws/src

RUN  python3 -m pip install transforms3d
CMD . /ws/install/setup.sh && artefacts run $ARTEFACTS_JOB_NAME