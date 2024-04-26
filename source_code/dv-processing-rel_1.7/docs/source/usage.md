# Usage in a project

The library can be used in C++ CMake projects after a successful installation. Python projects support conda and venv
environments using pip installation, pip installation is the preferred method for use in python, although system-wide
installation packages are also provided for some linux distributions.

## C++ (CMake)

To use the installed version of dv-processing in your project, add these lines to your CMakeLists.txt:

```cmake
# Find installed dv-processing.
FIND_PACKAGE(dv-processing)

# Link your targets against the library
TARGET_LINK_LIBRARIES(your_target
        dv::processing
        ...)
```

## Python

After installing the dv-processing python bindings either using pip or system-wide installation, the library is included
using a simple import statement:

```python
import dv_processing as dv

print(dv.__version__)
```

```{note}
Pip installation is the only supported method for installation in a virtual python environment (conda / venv).
```
