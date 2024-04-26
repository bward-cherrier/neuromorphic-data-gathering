#! /bin/bash

LCOV_BIN_DIR=/home/rokas/Workspace/ext/lcov/bin/
GCOV_TOOL=/usr/bin/gcov-9

$LCOV_BIN_DIR/lcov --gcov-tool=$GCOV_TOOL -c  --directory . --output-file i.info
$LCOV_BIN_DIR/lcov --remove i.info -o f.info "/home/rokas/Workspace/inivation/dv-processing/tests*" "9/*" "/usr/include/*" "/usr/lib/*"
