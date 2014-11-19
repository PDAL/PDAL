#
# BPF driver CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/drivers/bpf)

#
# BPF Reader
#
set(srcs
    BpfHeader.cpp
    BpfReader.cpp
)

set(incs
    BpfHeader.hpp
    BpfReader.hpp
)

PDAL_ADD_DRIVER(reader bpf "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
