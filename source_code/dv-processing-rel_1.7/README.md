# dv-processing

Generic algorithms for event cameras.

# Installation via package manager

The library is available for installation with apt package manager in recent Ubuntu distributions.

## Package installation in Ubuntu 18.04

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt update
sudo apt install dv-processing
```

## Package installation in Ubuntu 20.04

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing
```

## Python bindings

Python bindings for dv-processing can also be installed via apt (with ppa enabled):

```bash
sudo apt install dv-processing-python
```

# Dependencies:

- Linux, MacOS X or Windows
- gcc >= 10.0 or clang >= 13 or Apple XCode >= 14.3
- libstdc++ >= 10.0 or Apple XCode libc++ >= 14.3
- Microsoft Visual Studio 2022 >= 17.6 with VCPKG
- cmake >= 3.22
- Boost >= 1.76
- OpenCV >= 4.2.0
- Eigen >= 3.4.0
- libcaer >= 3.3.14
- fmt >= 8.1.1
- lz4
- zstd
- OpenSSL
- Optional: libbacktrace (for better stack traces on error)

# API Documentation

The API documentation is available in HTML format, please open `docs/index.html` with your browser to access the
documentation.

## Install dependencies on Ubuntu 20.04

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install git gcc-10 g++-10 cmake boost-inivation libopencv-dev libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev libssl-dev
```

## Install dependencies on Ubuntu 18.04

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt-get update
sudo apt-get install git gcc-10 g++-10 cmake boost-inivation libopencv-dev libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev libssl-dev
```

# Installation

The use of library is possible using two approaches - system installation or as a git submodule.

## System wide installation

1. Clone the repository:

```bash
git clone https://gitlab.com/inivation/dv/dv-processing.git
cd dv-processing
```

2. Build and verify the library using unit tests:

```bash
mkdir build && cd build
CC=gcc-10 CXX=g++-10 cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j4 -s
make test
```

3. Install the headers:

```bash
sudo make install
```

4. Use in your cmake projects:

```cmake
FIND_PACKAGE(dv-processing REQUIRED)

# link your targets against the library
TARGET_LINK_LIBRARIES(your_target
	dv::processing
	...)
```

## Git submodule usage

1. Add the repository as a submodule in your project:

```bash
git submodule add https://gitlab.com/inivation/dv/dv-processing.git path/for/dv-processing
```

2. Use in your cmake project:

```cmake
ADD_SUBDIRECTORY(path/for/dv-processing EXCLUDE_FROM_ALL)

# link your targets against the library
TARGET_LINK_LIBRARIES(your_target
	dv::processing
	...)
```
