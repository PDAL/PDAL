#
# BPF driver CMake configuration
#

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
