###############################################################################
#
# src/drivers/liblas/CMakeLists.txt controls building of libPC library
#
# Copyright (c) 2011 Howard Butler <hobu@hobu.net>
#
###############################################################################

set(LAS_HEADERS
  LasHeaderReader.hpp
  LasHeaderWriter.hpp
)
  
set (LAS_SOURCES
  LasHeaderReader.cpp
  LasHeaderWriter.cpp
  Header.cpp
  Reader.cpp
  Writer.cpp)
  

FOREACH(file ${LAS_SOURCES})
        SET(LIBPC_SOURCES "${LIBPC_SOURCES};${CMAKE_CURRENT_SOURCE_DIR}/${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)
