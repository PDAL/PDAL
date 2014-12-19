# - Try to find the Python module NumPy
#
# This module defines: 
#  NUMPY_INCLUDE_DIR: include path for arrayobject.h

# Copyright (c) 2009-2012 Arnaud Barré <arnaud.barre@gmail.com>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (NUMPY_INCLUDE_DIR)
  SET(NUMPY_FIND_QUIETLY TRUE)
endif (NUMPY_INCLUDE_DIR)

# To set the variables PYTHON_EXECUTABLE
FIND_PACKAGE(PythonInterp QUIET REQUIRED)
FIND_PACKAGE(PythonLibs QUIET REQUIRED)

# Look for the include path
# WARNING: The variable PYTHON_EXECUTABLE is defined by the script FindPythonInterp.cmake
EXECUTE_PROCESS(COMMAND "${PYTHON_EXECUTABLE}" -c "import numpy; print (numpy.get_include()); print (numpy.version.version)"
                 OUTPUT_VARIABLE NUMPY_OUTPUT
                 ERROR_VARIABLE NUMPY_ERROR)
                 
IF(NOT NUMPY_ERROR)
  STRING(REPLACE "\n" ";" NUMPY_OUTPUT ${NUMPY_OUTPUT})
  LIST(GET NUMPY_OUTPUT 0 NUMPY_INCLUDE_DIR)
  LIST(GET NUMPY_OUTPUT 1 NUMPY_VERSION)
ENDIF(NOT NUMPY_ERROR)

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(NumPy DEFAULT_MSG NUMPY_VERSION NUMPY_INCLUDE_DIR)

MARK_AS_ADVANCED(NUMPY_INCLUDE_DIR)
