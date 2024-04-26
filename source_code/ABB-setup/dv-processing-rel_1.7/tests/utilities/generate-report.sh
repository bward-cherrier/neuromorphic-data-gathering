#! /bin/bash

LCOV_BIN_DIR=/home/rokas/Workspace/ext/lcov/bin/

INFO_FILES=($(find . -name f.info))
$LCOV_BIN_DIR/lcov --add-tracefile ${INFO_FILES[0]} -a ${INFO_FILES[1]} -o merged.info
for info in "${INFO_FILES[@]:2}"
do :
  $LCOV_BIN_DIR/lcov --add-tracefile merged.info -a ${info} -o merged.info
done
$LCOV_BIN_DIR/genhtml merged.info
