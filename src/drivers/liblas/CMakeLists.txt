###############################################################################
#
# src/drivers/liblas/CMakeLists.txt controls building of libPC library
#
# Copyright (c) 2011 Howard Butler <hobu@hobu.net>
#
###############################################################################


set (SOURCES 
  header.cpp
  reader.cpp
  writer.cpp)
  

FOREACH(file ${SOURCES})
        SET(LIBPC_SOURCES "${LIBPC_SOURCES};${CMAKE_CURRENT_SOURCE_DIR}/${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)
