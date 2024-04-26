class DvProcessing < Formula
  desc "Generic algorithms for event cameras."
  homepage "https://gitlab.com/inivation/dv/dv-processing/"
  url "https://release.inivation.com/processing/dv-processing-VERSION_REPLACE.tar.gz"
  sha256 "SHA256SUM_REPLACE"

  version "VERSION_REPLACE"

  depends_on "cmake" => :build
  depends_on "pkg-config" => :build
  depends_on "boost"
  depends_on "opencv"
  depends_on "eigen"
  depends_on "libcaer"
  depends_on "fmt"
  depends_on "lz4"
  depends_on "zstd"
  depends_on "openssl"

  def install
    args = ["-DENABLE_TESTS=1", "-DENABLE_UTILITIES=1", "-DENABLE_BENCHMARKS=0", "-DENABLE_SAMPLES=0", "-DENABLE_PYTHON=0"]

    system "cmake", ".", *std_cmake_args, *args
    system "make", "install"
  end
end
