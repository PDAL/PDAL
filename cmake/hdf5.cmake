#
# HDF5
#

set (HDF5_FIND_COMPONENTS "CXX")
find_package(HDF5 COMPONENTS CXX REQUIRED )
if(HDF5_FOUND)
    if(HDF5_INCLUDE_DIRS AND NOT HDF5_INCLUDE_DIR)
        set(HDF5_INCLUDE_DIR ${HDF5_INCLUDE_DIRS})
    endif()
    include_directories(${HDF5_INCLUDE_DIR})
    if (WIN32)
        add_definitions(-DH5_BUILT_AS_DYNAMIC_LIB=1)
    endif()
    add_definitions(-DHAVE_HDF5=1)
    set(PDAL_HAVE_HDF5 1)
endif()
