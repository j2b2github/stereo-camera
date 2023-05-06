### ----- INSTALL TENSORRT ----- ###
FROM nvcr.io/nvidia/tensorrt:21.10-py3

### ----- INSTALL OPENCV ----- ###
# Set timezone
RUN apt-get update
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get install -y tzdata

# Install required libraries
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
build-essential \
gdb \
unzip \
git \
wget \
pkg-config \
python3-dev \
python3-numpy \
libatlas-base-dev \
gfortran \
libeigen3-dev \
libjpeg-dev \
libtiff5-dev \
libpng-dev \
openexr \
ffmpeg \
libavcodec-dev \
libavformat-dev \
libswscale-dev \
libxvidcore-dev \
libx264-dev \
libxine2-dev \
libv4l-dev \
v4l-utils \
qt5-default \
libfontconfig1 \
mesa-common-dev \
libglu1-mesa-dev \
libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev \
libgstreamer-plugins-bad1.0-dev \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav \
gstreamer1.0-tools \
gstreamer1.0-x \
gstreamer1.0-alsa \
gstreamer1.0-gl \
gstreamer1.0-gtk3 \
gstreamer1.0-qt5 \
gstreamer1.0-pulseaudio

# Install Cmake
RUN cd /tmp && \
wget https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3-linux-x86_64.sh && \
chmod +x cmake-3.26.3-linux-x86_64.sh && \
./cmake-3.26.3-linux-x86_64.sh --prefix=/usr/local --exclude-subdir --skip-license && \
rm ./cmake-3.26.3-linux-x86_64.sh

# Install opencv
RUN cd /tmp && \
mkdir -p opencv && \
cd opencv && \
wget -O opencv-4.7.0.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.7.0.tar.gz && \
tar -zxvf opencv-4.7.0.tar.gz && \
wget -O opencv_contrib-4.7.0.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/4.7.0.tar.gz && \
tar -zxvf opencv_contrib-4.7.0.tar.gz && \
mkdir -p build && \
cd build && \
cmake \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=ON \
-D BUILD_DOCS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PACKAGE=OFF \
-D BUILD_EXAMPLES=OFF \
-D WITH_TBB=OFF \
-D WITH_IPP=OFF \
-D ENABLE_FAST_MATH=OFF \
-D WITH_1394=OFF \
-D WITH_GTK=OFF \
-D WITH_QT=ON \
-D WITH_OPENGL=ON \
-D WITH_V4L=ON \
-D WITH_FFMPEG=ON \
-D WITH_EIGEN=ON \
-D WITH_GSTREAMER=ON \
-D WITH_TESSERACT=OFF \
-D BUILD_JAVA=OFF \
-D BUILD_opencv_python3=ON \
-D BUILD_opencv_python2=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D OPENCV_SKIP_PYTHON_LOADER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.7.0/modules \
-D PYTHON3_PACKAGES_PATH=/usr/local/lib/python3.8/dist-packages \
-D WITH_XINE=ON \
-D BUILD_WITH_DEBUG_INFO=OFF \
-D WITH_CUDA=ON \
-D WITH_CUBLAS=ON \
-D WITH_CUFFT=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_FAST_MATH=ON \
-D CUDA_ARCH_BIN=7.5 \
-D CUDA_ARCH_PTX=7.5 \
-D WITH_NVCUVID=ON \
-D WITH_NVCUVENC=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
-D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.8.7.0 \
-D CUDNN_INCLUDE_DIR=/usr/include \
../opencv-4.7.0 && \
make -j$(nproc) && \
make install && \
ldconfig && \
rm -rf /tmp/opencv
