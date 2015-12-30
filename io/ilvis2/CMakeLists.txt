#
# ILVIS2 driver CMake configuration
#

set(objs "")

#
# ILVIS2 Reader
#
set(srcs
    Ilvis2Reader.cpp
)

set(incs
    Ilvis2Reader.hpp
)

if (PDAL_HAVE_LIBXML2)
    list(APPEND srcs Ilvis2MetadataReader.cpp)
    list(APPEND incs Ilvis2MetadataReader.hpp)
endif()

PDAL_ADD_DRIVER(reader ilvis2 "${srcs}" "${incs}" reader_objs)
set(objs ${objs} ${reader_objs})

set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
