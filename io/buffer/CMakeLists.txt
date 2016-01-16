#
# Buffer driver CMake configuration
#

#
# Buffer Reader
#

set(incs
    BufferReader.hpp
)

install(FILES ${incs} DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}")
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)

