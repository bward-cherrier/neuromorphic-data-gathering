# Copyright 2021 iniVation AG
# Distributed under the terms of the GNU General Public License v2

EAPI=7

inherit eutils cmake

DESCRIPTION="Generic algorithms for event cameras."
HOMEPAGE="https://gitlab.com/inivation/dv/${PN}/"

SRC_URI="https://release.inivation.com/processing/${P}.tar.gz"

LICENSE="Apache-2.0"
SLOT="0"
KEYWORDS="amd64 x86 arm64 arm"
IUSE="debug test python"

RDEPEND="!<dev-util/dv-runtime-1.5.3
	>=dev-libs/boost-1.78.0
	>=media-libs/opencv-4.5.5
	>=dev-cpp/eigen-3.4.0
	>=dev-libs/libcaer-3.3.14
	>=app-arch/lz4-1.9.0
	>=app-arch/zstd-1.5.0
	>=dev-libs/libfmt-8.1.1
	dev-libs/openssl
	python? (
		>=dev-lang/python-3.8.0
		dev-python/numpy
	)"

DEPEND="${RDEPEND}
	virtual/pkgconfig
	>=dev-util/cmake-3.22.0"

src_configure() {
	local mycmakeargs=(
		-DENABLE_TESTS="$(usex test 1 0)"
		-DENABLE_UTILITIES=1
		-DENABLE_PYTHON="$(usex python 1 0)"
		-DENABLE_BENCHMARKS=0
		-DENABLE_SAMPLES=0
	)

	cmake_src_configure
}
