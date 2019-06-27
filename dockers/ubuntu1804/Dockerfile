FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
ENV LANG C.UTF-8
RUN apt-get update && apt-get install -y git
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update
RUN apt-get install -y gcc-8 g++-8
ENV CXX=/usr/bin/g++-8
ENV CC=/usr/bin/gcc-8

# ==================================================================
# create working directories
# ------------------------------------------------------------------
ENV WORKSPACE=/raisim_workspace
ENV LOCAL_BUILD=/raisim_build
RUN mkdir -p $WORKSPACE
RUN mkdir -p $LOCAL_BUILD

# ==================================================================
# tools
# ------------------------------------------------------------------
RUN apt-get install -y cmake python3.6-dev python3-pip libpython3.6-dev

# ==================================================================
# tensorflow
# ------------------------------------------------------------------
RUN pip3 install tensorflow-gpu setuptools

# ==================================================================
# raisim
# ------------------------------------------------------------------
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimLib.git
WORKDIR $WORKSPACE/raisimLib
RUN mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD && make install

# ==================================================================
# raisimOgre
# ------------------------------------------------------------------
# ogre
RUN apt-get install -y libgles2-mesa-dev libxt-dev libxaw7-dev libsdl2-dev libzzip-dev
RUN cd $WORKSPACE && git clone https://github.com/OGRECave/ogre.git
WORKDIR $WORKSPACE/ogre
RUN git checkout tags/v1.11.5 -b r1.11.5 && mkdir build && cd build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD && make install -j8
RUN cp -R Dependencies/* $LOCAL_BUILD

# assimp
RUN cd $WORKSPACE && git clone https://github.com/assimp/assimp.git
WORKDIR $WORKSPACE/assimp
RUN mkdir build && cd build
RUN cmake .. -G 'Unix Makefiles' -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD && make install -j8

# raisimOgre
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimOgre.git
RUN mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release\
                                        -DCMAKE_PREFIX_PATH=$LOCAL_BUILD\
                                        -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD
RUN make install -j

# ==================================================================
# raisimGym
# ------------------------------------------------------------------
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimGym.git
WORKDIR $WORKSPACE/raisimGym

# ==================================================================
# display
# ------------------------------------------------------------------
# For nvidia GUI issues
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

EXPOSE 6006