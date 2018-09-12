FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu16.04
RUN apt-get update && apt-get install -y wget&&\
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
       libboost-all-dev=1.54.0.1ubuntu1 &&\
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        libatlas-base-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libhdf5-serial-dev \
        libleveldb-dev \
        liblmdb-dev \
        libopencv-dev \
        libprotobuf-dev \
        libsnappy-dev \
        protobuf-compiler \
        python-dev \
        python-numpy \
        python-pip \
        python-setuptools \
        python-scipy && \
    rm -rf /var/lib/apt/lists/*

# Installation of ROS
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA117
RUN apt-get update && apt-get install -y vim\
    libeigen3-dev\
    ros-kinetic-desktop-full\
    ros-kinetic-cv-bridge\
    python-tk\
    x11-apps\
    ros-kinetic-rqt-rviz\
    ros-kinetic-pcl-ros\
    ros-kinetic-tf-conversions\
    ros-kinetic-rqt-graph&&\
    rm -rf /var/lib/apt/lists/*

# Installation of Caffe
WORKDIR /opt
RUN mkdir caffe
COPY caffe caffe
ENV CAFFE_ROOT=/opt/caffe
WORKDIR $CAFFE_ROOT
RUN mkdir build
WORKDIR $CAFFE_ROOT/build
RUN cmake -DUSE_CUDNN=1 -D CMAKE_BUILD_TYPE=RELEASE ..
RUN make -j8
ENV PYCAFFE_ROOT $CAFFE_ROOT/python
ENV PYTHONPATH $PYCAFFE_ROOT:$PYTHONPATH
ENV PATH $CAFFE_ROOT/build/tools:$PYCAFFE_ROOT:$PATH
RUN echo "$CAFFE_ROOT/build/lib" >> /etc/ld.so.conf.d/caffe.conf && ldconfig

WORKDIR /opt
COPY gpg gpg
RUN mkdir gpg/build
WORKDIR gpg/build
RUN cmake ..
RUN make -j8
RUN make install

RUN pip install pytest

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
COPY gpd gpd
WORKDIR /catkin_ws

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

ENTRYPOINT ["devel/env.sh"]
CMD ["roslaunch", "gpd", "ur5_15_channels.launch"]
