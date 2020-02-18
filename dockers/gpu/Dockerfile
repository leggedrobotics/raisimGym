FROM nvidia/cudagl:10.1-base-ubuntu18.04
ENV LANG C.UTF-8
RUN apt-get update && apt-get install -y git
RUN apt-get install -y software-properties-common
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
RUN apt-get install -y cmake python3.6-dev python3-pip libpython3.6-dev libeigen3-dev

# ==================================================================
# tensorflow
# ------------------------------------------------------------------
RUN pip3 install tensorflow-gpu==1.14 setuptools

# ==================================================================
# raisim
# ------------------------------------------------------------------
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimLib.git
RUN cd $WORKSPACE/raisimLib && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD && make install

# ==================================================================
# raisimOgre
# ------------------------------------------------------------------
# ogre
RUN apt-get update && apt-get -y install libgles2-mesa-dev libxt-dev libxaw7-dev libsdl2-dev libzzip-dev libfreeimage-dev libfreetype6-dev libpugixml-dev
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/ogre.git
WORKDIR $WORKSPACE/ogre
RUN git checkout raisimOgre && mkdir build && cd build
WORKDIR $WORKSPACE/ogre/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DOGRE_BUILD_COMPONENT_BITES=ON -OGRE_BUILD_COMPONENT_JAVA=OFF -DOGRE_BUILD_DEPENDENCIES=OFF -DOGRE_BUILD_SAMPLES=False && make install -j8

# raisimOgre
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimOgre.git && cd raisimOgre && mkdir build
WORKDIR $WORKSPACE/raisimOgre/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release\
             -DCMAKE_PREFIX_PATH=$LOCAL_BUILD\
             -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD
RUN make install -j12

# ==================================================================
# raisimGym
# ------------------------------------------------------------------
# pybind11
RUN cd $WORKSPACE && git clone https://github.com/pybind/pybind11.git &&\
    cd pybind11 && git checkout v2.4.3 && mkdir build && cd build &&\
    cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_BUILD -DPYBIND11_TEST=OFF &&\
    make install -j4

# raisimGym
RUN apt-get -y install libyaml-cpp-dev libopenmpi-dev zlib1g-dev python3-pip
RUN pip3 install ruamel.yaml
RUN cd $WORKSPACE && git clone https://github.com/leggedrobotics/raisimGym.git

# ==================================================================
# add ld path
# ------------------------------------------------------------------
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$LOCAL_BUILD/lib"

# ==================================================================
# run test
# ------------------------------------------------------------------
WORKDIR $WORKSPACE/raisimGym

# ===================================================================
# docker-speicific
# ------------------------------------------------------------------
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# ==================================================================
# firefox
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y firefox

# ==================================================================
# display
# ------------------------------------------------------------------
# For nvidia GUI issues
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64
EXPOSE 6006
