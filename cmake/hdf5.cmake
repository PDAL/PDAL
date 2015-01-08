#
# HDF5
#

set (HDF5_FIND_COMPONENTS "CXX")
find_package(HDF5 COMPONENTS CXX REQUIRED)
if(HDF5_FOUND)
    include_directories(${HDF5_INCLUDE_DIR})
    add_definitions(-DHAVE_HDF5=1)
    set(PDAL_HAVE_HDF5 1)
endif()
