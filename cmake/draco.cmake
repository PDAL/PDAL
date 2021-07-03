#
# the `pkg_check_modules` function is created with this call
find_package(PkgConfig REQUIRED)
pkg_check_modules(DRACO REQUIRED draco>1.4.0)

set(PDAL_HAVE_DRACO 1)
