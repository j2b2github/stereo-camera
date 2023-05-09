### ----- INSTALL TENSORRT ----- ###
FROM nvcr.io/nvidia/tensorrt:21.10-py3

### ----- INSTALL OPENCV ----- ###
ENV OPENCV_VERSION 4.7.0

# Set timezone
RUN apt-get update
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get install -y tzdata

# Install required libraries
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y build-essential gdb unzip git curl wget pkg-config nano tmux htop mc
RUN apt-get install -y python3-dev python3-numpy 
RUN apt-get install -y gfortran libeigen3-dev
RUN apt-get install -y libglu1-mesa-dev mesa-common-dev mesa-utils
RUN apt-get install -y qt5-default
RUN apt-get install -y libjpeg-dev libtiff-dev libpng-dev libopenexr-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libv4l-dev v4l-utils

# Install Cmake
WORKDIR /tmp
RUN wget https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3-linux-x86_64.sh
RUN chmod +x cmake-3.26.3-linux-x86_64.sh
RUN ./cmake-3.26.3-linux-x86_64.sh --prefix=/usr/local --exclude-subdir --skip-license
RUN rm ./cmake-3.26.3-linux-x86_64.sh

# download opencv source
RUN mkdir -p /opt/opencv
WORKDIR /opt/opencv
RUN wget -O opencv-$OPENCV_VERSION.tar.gz https://github.com/opencv/opencv/archive/refs/tags/$OPENCV_VERSION.tar.gz
RUN tar -zxvf opencv-$OPENCV_VERSION.tar.gz && rm -f opencv-$OPENCV_VERSION.tar.gz
RUN wget -O opencv_contrib-$OPENCV_VERSION.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/$OPENCV_VERSION.tar.gz
RUN tar -zxvf opencv_contrib-$OPENCV_VERSION.tar.gz && rm -f opencv_contrib-$OPENCV_VERSION.tar.gz

# Install opencv
RUN mkdir -p /opt/opencv/build
WORKDIR /opt/opencv/build
RUN cmake \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-$OPENCV_VERSION/modules \
-D OPENCV_ENABLE_NONFREE=ON \
-D WITH_EIGEN=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_WITH_DEBUG_INFO=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PACKAGE=OFF \
-D BUILD_EXAMPLES=OFF \
-D WITH_1394=OFF \
-D WITH_GTK=OFF \
-D WITH_QT=ON \
-D WITH_OPENGL=ON \
-D WITH_V4L=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=OFF \
-D WITH_TESSERACT=OFF \
-D BUILD_JAVA=OFF \
-D PYTHON3_PACKAGES_PATH=/usr/local/lib/python3.8/dist-packages \
-D BUILD_opencv_python3=ON \
-D BUILD_opencv_python2=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D OPENCV_SKIP_PYTHON_LOADER=ON \
-D WITH_CUDA=ON \
-D WITH_CUBLAS=ON \
-D WITH_CUFFT=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_FAST_MATH=ON \
-D CUDA_ARCH_BIN=7.5 \
-D CUDA_ARCH_PTX=7.5 \
-D WITH_NVCUVID=ON \
-D WITH_NVCUVENC=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
-D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.8.7.0 \
-D CUDNN_INCLUDE_DIR=/usr/include \
../opencv-$OPENCV_VERSION
RUN make -j$(nproc)
RUN make install
RUN ldconfig

WORKDIR /workspace/