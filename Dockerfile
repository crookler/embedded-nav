FROM ubuntu:24.04

# Apparently libeigen3-dev is a halfway decent linear algebra library
# May need to find some others
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev 