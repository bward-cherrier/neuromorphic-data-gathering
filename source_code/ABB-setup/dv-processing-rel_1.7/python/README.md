## Python bindings for _dv::processing_

This directory contains Python bindings for dv::processing. These are disabled by default and can be enabled by setting
the Cmake option `ENABLE_PYTHON` to `ON` when configuring the cmake project:

```bash
cmake .. -DENABLE_PYTHON=ON
```

### Dependencies

**Bindings require Python >= 3.6**

On Linux, you need to install a few dependencies:

```shell
sudo apt install python-dev python-numpy
```

or

```shell
sudo apt install python3-dev python3-numpy
```

depending on your Python version.

### Testing

To run all available tests run `make test`. To run python tests only, it can be done using the command:

```bash
ctest -R test-python-bindings
```

### Installation

Installation is performed using the regular `sudo make install` command.

### Stub generation

Stubs are a way of generating function signatures that can be used by IDEs and other packages to provide code completion
and suggestions. Stubs can be generated using `pybind11-stubgen` package, it can be installed from `pip`:

```bash
pip install pybind11-stubgen
```

If the package is installed, stubs will be generated after building the bindings library and installed in the system.
