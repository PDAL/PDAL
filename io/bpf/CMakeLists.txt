#
# BPF driver CMake configuration
#

#
# BPF Reader
#
set(srcs
    BpfHeader.cpp
    BpfReader.cpp
    BpfWriter.cpp
)

set(incs
    BpfHeader.hpp
    BpfReader.hpp
    BpfWriter.hpp
)

PDAL_ADD_DRIVER(reader bpf "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
