#
# Python
#

#
# Version 3.12 has shiny new FindPython3 scripts
# Version 3.14 introduces direct support for NumPy.
# When we require newer cmake versions, ditch the older support.
#
if (NOT (CMAKE_VERSION VERSION_LESS "3.14.0"))
    find_package(Python3 COMPONENTS Interpreter Development NumPy)
    set(PYTHON_LIBRARY ${Python3_LIBRARIES}
        CACHE FILEPATH "Python library")
    set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS}
        CACHE PATH "Location of Python include files.")
    set(PYTHON_NUMPY_INCLUDE_DIR ${Python3_NumPy_INCLUDE_DIRS}
        CACHE PATH "Location of NumPy include files.")
    set(PDAL_HAVE_PYTHON 1)
elseif (NOT (CMAKE_VERSION VERSION_LESS "3.12.0"))
    find_package(Python3 COMPONENTS Interpreter Development)
    set(PYTHON_LIBRARY ${Python3_LIBRARIES}
        CACHE FILEPATH "Python library")
    set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS}
        CACHE PATH "Location of Python include files.")
     set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
     find_package(NumPy 1.5 REQUIRED)
    set(PDAL_HAVE_PYTHON 1)
else()
    find_package(PythonInterp 3)
    find_package(PythonLibs 3)
    set(PDAL_HAVE_PYTHON 1)
    find_package(NumPy 1.13 REQUIRED)
endif()
set(PYTHON_ALL_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR} ${PYTHON_NUMPY_INCLUDE_DIR})
