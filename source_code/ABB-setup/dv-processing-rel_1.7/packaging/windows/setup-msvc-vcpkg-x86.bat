Rem Call Visual Studio setup script, will setup all paths and other needed environment variables.

call "%ProgramFiles%\Microsoft Visual Studio\2022\Preview\VC\Auxiliary\Build\vcvars32.bat"

Rem Set VCPKG / CMake environment variables.

set PATH=%PATH%;C:\vcpkg\installed\x86-windows\bin
set CMAKE_ARGS=-DVCPKG_TARGET_TRIPLET=x86-windows -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake

set

Rem Run cibuildwheel to build Python wheels for x86.

python3 -m cibuildwheel --platform windows --archs x86 --output-dir wheelhouse
