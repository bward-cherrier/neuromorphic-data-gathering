# Installation of dv-processing

The dv-processing library can be installed using different package management tools as well as installations from
source. Since it is a header-only library, it can be used as a submodule in other CMake C++ projects.

## C++

Installation of dv-processing headers is available on Linux, MacOS, and Windows platforms.

### Linux {fa}`linux`

#### Ubuntu 18.04 (apt)

```shell
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt-get update
sudo apt-get install dv-processing
```

#### Ubuntu 20.04 / 22.04 (apt)

```shell
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install dv-processing
```

#### Fedora Linux

```shell
sudo dnf copr enable inivation/inivation
sudo dnf install dv-processing
```

#### Arch Linux

You can find dv-processing in the AUR repository, install the package 'dv-processing'.

The standard packages in the AUR repository already include all development files.

#### Gentoo Linux

A valid Gentoo ebuild repository is available [here](https://gitlab.com/inivation/gentoo-inivation/) over Git. The
package to install is 'dev-libs/dv-processing'.

The standard packages in the Gentoo ebuild repository already include all development files.

### MacOS (brew) {fa}`apple`

Please notice that MacOS installation requires a recent compiler version (Apple XCode >= 14.3 or LLVM >= 13).

```shell
brew tap inivation/inivation
brew install libcaer --with-libserialport --with-opencv
brew install dv-processing
```

### Windows (VCPKG) {fa}`windows`

Windows installation is supported using the VCPKG package manager. This requires cloning of the VCPKG repository, you
can follow the official [quick start guide to get started](https://github.com/microsoft/vcpkg#quick-start-windows). The
library can be installed using `.\vcpkg.exe install dv-processing` after VCPKG has been downloaded and bootstrapped.

A short tutorial for installation, create a directory at `C:\src`, execute these commands there:

```shell
git clone https://github.com/microsoft/vcpkg
.\vcpkg\bootstrap-vcpkg.bat
.\vcpkg\vcpkg install dv-processing
```

Installing `dv-processing[tools]` will also compile and install the command-line utilities, such as dv-filestat.

### Installing as a git submodule

The library can be used in a CMake project by adding it as a git submodule. To add the library using git, call the
following command from your project directory:

```shell
git submodule https://gitlab.com/inivation/dv/dv-processing.git thirdparty/dv-processing
```

This will add the source code of the latest released version of dv-processing to a directory `thirdparty/dv-processing`.
Now you can enable it in your CMakeLists.txt:

```cmake
ADD_SUBDIRECTORY(thirdparty/dv-processing EXCLUDE_FROM_ALL)

# link your targets against the library
TARGET_LINK_LIBRARIES(your_target
	dv::processing
	...)
```

### Build and install from source

Manual build and installation from source is also possible. The library requires a C++20 compatible compiler and
installed dependencies.

Please follow the installation instructions available in the
[source code repository](https://gitlab.com/inivation/dv/dv-processing/) to install the library from source.

## Python {fa}`python`

The dv-processing library is also available in Python using Pybind11 bindings. Since the underlying implementation
remains C++ and only exposes the API through Python bindings, the performance of the provided methods remains high.

Installation using `pip install` is recommended, as it works on all supported operating systems, as well as VirtualEnv
and Conda environments. On Linux systems, installation can also be performed through the package manager for the system
Python installation.

### Pip

```shell
python3 -m pip install dv-processing
```

```{note}
Pip installation is the only supported method for installation in a virtual Python environment (VirtualEnv / Conda).
```

### Linux {fa}`linux`

#### Ubuntu 18.04 (apt)

```shell
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt-get update
sudo apt-get install dv-processing-python
```

#### Ubuntu 20.04 / 22.04 (apt)

```shell
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install dv-processing-python
```

#### Fedora Linux

```shell
sudo dnf copr enable inivation/inivation
sudo dnf install dv-processing-python
```

#### Arch Linux

You can find dv-processing in the AUR repository, install the package 'dv-processing', which includes the Python
bindings.

#### Gentoo Linux

A valid Gentoo ebuild repository is available [here](https://gitlab.com/inivation/gentoo-inivation/) over Git. The
package to install is 'dev-libs/dv-processing' with the USE flag 'python' enabled.
