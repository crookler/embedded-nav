FROM ubuntu:24.04

# Apparently libeigen3-dev is a halfway decent linear algebra library
# Trying out Matplot++ for visualization strategy
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    git \
    gnuplot \
    libpng-dev 

# Need to build Matplot++ from source (copied from documentation but also need git)
RUN git clone https://github.com/alandefreitas/matplotplusplus/ /matplotplusplus && \
    cd /matplotplusplus && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_TESTS=OFF && \
    cmake --build . --parallel 2 --config Release && \
    cmake --install . && \
    rm -rf /matplotplusplus