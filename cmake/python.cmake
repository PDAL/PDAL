#
# Python
#

#
# Version 3.12 has shiny new FindPython2 and FindPython3 scripts
#
if (NOT (CMAKE_VERSION VERSION_LESS "3.12.0"))
    find_package(Python3 QUIET COMPONENTS Interpreter Development NumPy)
    if (NOT PYTHON3_FOUND)
        find_package(Python2 2.7 REQUIRED
            COMPONENTS Interpreter Development Numpy)

        # Since we've required 2.7, these should all be valid
        set(PYTHON_LIBRARIES ${Python2_LIBRARIES})
        set(PYTHON_INCLUDE_DIRS ${Python2_INCLUDE_DIRS})
        set(PYTHON_NUMPY_INCLUDE_DIRS ${Python2_NumPy_INCLUDE_DIRS})
    else()
        set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
        set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
        set(PYTHON_NUMPY_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS})
    endif()
    set(PDAL_HAVE_PYTHON 1)
else()
    find_package(PythonInterp 3 QUIET)
    find_package(PythonLibs 3 QUIET)
    if ((NOT PYTHONINTERP_FOUND) OR (NOT PYTHONLIBS_FOUND))
        unset(PYTHON_EXECUTABLE CACHE)
        find_package(PythonInterp 2.7 EXACT REQUIRED)
        find_package(PythonLibs 2.7 EXACT REQUIRED)
    endif()
    set(PDAL_HAVE_PYTHON 1)
    find_package(NumPy 1.5 REQUIRED)
endif()
