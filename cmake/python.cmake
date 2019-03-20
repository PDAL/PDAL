#
# Python
#

# Try to find version 3.  If not, we fall back to version 2.
find_package(PythonInterp QUIET 3)
find_package(PythonInterp QUIET 2.7)

# Looking for PythonLibs will use the version of the intpreter found to
# tell it which version of the libraries to use.
find_package(PythonLibs QUIET)

set_package_properties(PythonInterp PROPERTIES TYPE REQUIRED)
if(PYTHONLIBS_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES "${PYTHON_LIBRARY}")
    include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
    add_definitions(-DHAVE_PYTHON=1)
    set(PDAL_HAVE_PYTHON 1)
    set(PDAL_PYTHON_VERSION_STRING "${PYTHONLIBS_VERSION_STRING}" CACHE STRING "PDAL Python version" FORCE)

    find_package(NumPy QUIET 1.5 REQUIRED)
    include_directories(SYSTEM ${NUMPY_INCLUDE_DIR})
endif()
