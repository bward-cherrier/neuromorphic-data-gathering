#!/usr/bin/env sh

# exit when any command fails
set -e

BRANCH="master"

if [ -n "$1" ] ; then
	BRANCH="$1"
fi

mkdir libcaer_build
cd libcaer_build

git clone "https://gitlab-ci-token:${CI_JOB_TOKEN}@gitlab.com/inivation/dv/internal/libcaer-internal.git" .
git checkout "$BRANCH"

mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_OPENCV=1 ..
make -j2 -s
make install

cd ../..
rm -Rf libcaer_build
