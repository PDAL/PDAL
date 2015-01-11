#
# Python
#
find_package(PythonInterp QUIET )
find_package(PythonLibs QUIET 2.4)
set_package_properties(PythonInterp PROPERTIES TYPE REQUIRED)
if(PYTHONLIBS_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES "${PYTHON_LIBRARY}")
    include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
    add_definitions(-DHAVE_PYTHON=1)
    set(PDAL_HAVE_PYTHON 1)

    find_package(NumPy QUIET 1.5 REQUIRED)
    include_directories(SYSTEM ${NUMPY_INCLUDE_DIR})
endif()
