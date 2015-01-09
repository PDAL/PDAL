#
# SBET driver CMake configuration
#

set(objs "")

add_library(sbetcommon OBJECT SbetCommon.cpp SbetCommon.hpp)
set(objs ${objs} $<TARGET_OBJECTS:sbetcommon>)

#
# SBET Reader
#
set(srcs
    SbetReader.cpp
)

set(incs
    SbetReader.hpp
)

PDAL_ADD_DRIVER(reader sbet "${srcs}" "${incs}" reader_objs)
set(objs ${objs} ${reader_objs})

#
# SBET Writer
#
set(srcs
    SbetWriter.cpp
)

set(incs
    SbetWriter.hpp
)

PDAL_ADD_DRIVER(writer sbet "${srcs}" "${incs}" writer_objs)
set(objs ${objs} ${writer_objs})

set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
