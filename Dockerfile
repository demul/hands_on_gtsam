FROM ubuntu:18.04

RUN sed -i 's@archive.ubuntu.com@mirror.kakao.com@g' /etc/apt/sources.list
RUN apt update --fix-missing
RUN apt-get update --fix-missing
RUN apt install -y --no-install-recommends wget unzip git ca-certificates python3-dev ssh build-essential gcc-8 g++-8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
RUN apt install -y --no-install-recommends autoconf libtool
RUN apt-get install -y --no-install-recommends libboost-all-dev
RUN apt install -y --no-install-recommends pkg-config
RUN apt-get install -y --no-install-recommends python3-pip libgl1-mesa-glx libgtk2.0-dev
RUN apt install -y --no-install-recommends libmysqlcppconn-dev
RUN apt-get install -y --no-install-recommends libyaml-cpp-dev gdb
RUN apt install -y --no-install-recommends clang-format
RUN apt-get install -y libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev

## CMake
WORKDIR /root/
RUN wget -q -O cmake-linux.sh https://github.com/Kitware/CMake/releases/download/v3.19.6/cmake-3.19.6-Linux-x86_64.sh
RUN sh cmake-linux.sh -- --skip-license --prefix=/usr
RUN rm cmake-linux.sh

## argparse
WORKDIR /root/
RUN wget -O v2.9.zip https://github.com/p-ranav/argparse/archive/refs/tags/v2.9.zip
RUN unzip v2.9.zip && \
    rm v2.9.zip && \
    cd argparse-2.9 && \
    mkdir build
WORKDIR /root/argparse-2.9/build/
RUN cmake -DARGPARSE_BUILD_SAMPLES=on -DARGPARSE_BUILD_TESTS=on ..
RUN make install

## Python
RUN pip3 install numpy setuptools 
RUN pip3 install pandas scipy black
RUN pip3 install PyYaml psutil

## OpenCV
WORKDIR /root/
RUN wget -O opencv-3.4.13.zip https://github.com/opencv/opencv/archive/3.4.13.zip --no-check-certificate
RUN unzip -q opencv-3.4.13.zip && \
    rm opencv-3.4.13.zip && \
    cd opencv-3.4.13 && \
    mkdir build
WORKDIR /root/opencv-3.4.13/build/
RUN cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr\
    -D WITH_NVCUVID=OFF \
    -D WITH_TBB=ON \
    -D BUILD_TBB=ON \
    -D WITH_OPENMP=ON \
    -D WITH_V4L=ON \
    -D WITH_LIBV4L=ON \
    -D WITH_NVCUVID=OFF \
    -D BUILD_JPEG=ON \
    -D BUILD_TESTS=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH=/usr/lib/python3/dist-packages \
    -D PYTHON_EXECUTABLE=$(which python3) \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D WITH_GSTREAMER=OFF \
    -D WITH_GSTREAMER_0_10=OFF \
    -D WITH_CUDA=OFF -D \
    INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_TESTS=OFF ..
RUN make -j4 install

## Copy repository
COPY . /root/

## Eigen
WORKDIR /root/hands_on_gtsam/third_party/eigen/
RUN mkdir build
WORKDIR /root/hands_on_gtsam/third_party/eigen/build/
RUN cmake ..
RUN make install

## GTSAM
WORKDIR /root/hands_on_gtsam/third_party/gtsam/
RUN mkdir build
WORKDIR /root/hands_on_gtsam/third_party/gtsam/build/
########## TODO : ##########
# -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF 플래그는 20~30% 정도의 성능저하를 유발한다.
# GTSAM에서 해당 플래그를 의존패키지들에 자동으로 상속시켜주지 못하는 문제이며, OFF로 놓는 것은 임시방편임.
# https://neubility.atlassian.net/browse/ATNM-1209
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
############################
RUN make install

## OpenGV
WORKDIR /root/hands_on_gtsam/third_party/opengv/
RUN mkdir build
WORKDIR /root/hands_on_gtsam/third_party/opengv/build
RUN cmake .. -DEIGEN_INCLUDE_DIR="/usr/local/include/eigen3" -DBUILD_TESTS=ON -DCMAKE_INSTALL_PREFIX="/usr/local"
RUN make install

WORKDIR /root/hands_on_gtsam
