###############################################################################
#
# src/drivers/liblas/CMakeLists.txt controls building of libPC library
#
# Copyright (c) 2011 Howard Butler <hobu@hobu.net>
#
###############################################################################


set (SOURCES 
  oci_wrapper.cpp
  common.cpp
  header.cpp
  reader.cpp
  writer.cpp
  
  )

SET(APPS_CPP_DEPENDENCIES "${APPS_CPP_DEPENDENCIES};${ORACLE_LIBRARY}" CACHE INTERNAL "source files for foo")


FOREACH(file ${SOURCES})
        SET(LIBPC_SOURCES "${LIBPC_SOURCES};${CMAKE_CURRENT_SOURCE_DIR}/${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)
