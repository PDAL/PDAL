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
    set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
    set(PDAL_HAVE_PYTHON 1)
elseif (NOT (CMAKE_VERSION VERSION_LESS "3.12.0"))
    find_package(Python3 COMPONENTS Interpreter Development)
    set(PYTHON_LIBRARY ${Python3_LIBRARIES}
        CACHE FILEPATH "Python library")
    set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS}
        CACHE PATH "Location of Python include files.")
    set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
    find_package(NumPy 1.13 REQUIRED)
    set(PDAL_HAVE_PYTHON 1)
else()
    find_package(PythonInterp 3)
    find_package(PythonLibs 3)
    set(PDAL_HAVE_PYTHON 1)
    find_package(NumPy 1.13 REQUIRED)
endif()

# Deal with statically linked Pythons

execute_process(
  COMMAND
  ${PYTHON_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_config_var('Py_ENABLE_SHARED'))"
  OUTPUT_VARIABLE Py_ENABLE_SHARED
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT WIN32)
# See https://bugs.python.org/msg277944
# The "command to create shared modules". Used as variable in the "Makefile (and similar) templates to build python modules"
# for both in-python and third party modules. Initialized to hold the value which works for third party modules to link
# against the _installed_ python.
# We strip off the first word though (which will be the compiler name).
execute_process(
    COMMAND
    ${PYTHON_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_config_var('LDSHARED').split(' ', 1)[1])"
    OUTPUT_VARIABLE PYTHON_LDSHARED
    OUTPUT_STRIP_TRAILING_WHITESPACE
 )
 message("PYTHON Py_ENABLE_SHARED: ${Py_ENABLE_SHARED}")
 message("PYTHON USING LINK LINE: ${PYTHON_LDSHARED}")
endif()

set(PYTHON_LDSHARED ${PYTHON_LDSHARED}
    CACHE PATH "Python LDSHARED linking line")
set(PYTHON_IS_SHARED_ENABLED ${Py_ENABLE_SHARED}
    CACHE PATH "Was Python dynamically linked?")

if (APPLE)
    if (NOT Py_ENABLE_SHARED)
        # conda gives us -bundle, which isn't valid
        string(REPLACE  "-bundle" "" PYTHON_LDSHARED "${PYTHON_LDSHARED}")
        string(STRIP  ${PYTHON_LDSHARED} PYTHON_LDSHARED)
        set(PYTHON_LIBRARY ${PYTHON_LDSHARED})
    endif()
endif()


set(PYTHON_ALL_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR} ${PYTHON_NUMPY_INCLUDE_DIR})
