%global __cmake_in_source_build 1
%global debug_package %{nil}

Summary: Generic algorithms for event cameras (C++ headers)
Name:    dv-processing
Version: VERSION_REPLACE
Release: 0%{?dist}
License: ASL 2.0
URL:     https://gitlab.com/inivation/dv/dv-processing/
Vendor:  iniVation AG

Source0: https://release.inivation.com/processing/%{name}-%{version}.tar.gz

BuildRequires: gcc >= 10.0, gcc-c++ >= 10.0, cmake >= 3.22, pkgconfig >= 0.29.0, boost-devel >= 1.76, opencv-devel >= 4.5.0, eigen3-devel >= 3.4.0, libcaer-devel >= 3.3.14, fmt-devel >= 8.1.1, lz4-devel, libzstd-devel, openssl-devel, python3-devel, python3-numpy
Requires: cmake >= 3.22, pkgconfig >= 0.29.0, boost-devel >= 1.76, opencv-devel >= 4.5.0, eigen3-devel >= 3.4.0, libcaer-devel >= 3.3.14, fmt-devel >= 8.1.1, lz4-devel, libzstd-devel, openssl-devel

%description
Generic algorithms for event cameras (C++ headers).

%package utils
Summary: Generic algorithms for event cameras (CLI utilities)
Requires: boost >= 1.76, opencv >= 4.5.0, libcaer >= 3.3.14, fmt >= 8.1.1, lz4-libs, libzstd, openssl
Conflicts: dv-runtime < 1.5.3

%description utils
Generic algorithms for event cameras (CLI utilities).

%package python
Summary: Generic algorithms for event cameras (Python bindings)
Requires: boost >= 1.76, opencv >= 4.5.0, libcaer >= 3.3.14, fmt >= 8.1.1, lz4-libs, libzstd, openssl, python3, python3-numpy

%description python
Generic algorithms for event cameras (Python bindings).

%prep
%autosetup

%build
%cmake -DENABLE_TESTS=1 -DENABLE_UTILITIES=1 -DENABLE_PYTHON=1 -DENABLE_BENCHMARKS=0 -DENABLE_SAMPLES=0
%cmake_build

%install
export QA_RPATHS=$(( 0x0001|0x0010 ))
%cmake_install

%files
%{_includedir}/dv-processing/
%{_libdir}/pkgconfig/
%{_libdir}/cmake/dv-processing/

%files utils
%{_bindir}/

%files python
%{python3_sitearch}/

%changelog
* Mon Apr 20 2020 iniVation AG <support@inivation.com> - VERSION_REPLACE
- See ChangeLog file in source or GitLab.
