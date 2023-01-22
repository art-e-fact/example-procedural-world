FROM public.ecr.aws/artefacts/ros2:humble-dev

RUN wget https://mirror.clarkson.edu/blender/release/Blender3.4/blender-3.4.1-linux-x64.tar.xz && \
  tar -xvf blender-3.4.1-linux-x64.tar.xz --strip-components=1 -C /bin && \
  rm -rf blender-3.4.1-linux-x64.tar.xz && \
  rm -rf blender-3.4.1-linux-x64
RUN apt install python3-transforms3d
RUN python3 -m pip install plotly

# COPY ./artefacts-client ./artefacts-client 
# RUN pip install --editable ./artefacts-client

COPY . /ws/src

WORKDIR /ws

RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential

WORKDIR /ws/src

CMD . /ws/install/setup.sh && artefacts run $ARTEFACTS_JOB_NAME