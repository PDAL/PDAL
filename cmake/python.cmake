#
# Python
#

#
# Version 3.12 has shiny new FindPython2 and FindPython3 scripts
# Version 3.14 introduces direct support for NumPy.
# When we require newer cmake versions, ditch the older support.
#
if (NOT (CMAKE_VERSION VERSION_LESS "3.14.0"))
    find_package(Python3 COMPONENTS Interpreter Development NumPy)
    if (NOT Python3_FOUND)
        find_package(Python2 2.7 REQUIRED EXACT
            COMPONENTS Interpreter Development NumPy)

        # Since we've required 2.7, these should all be valid
        set(PYTHON_LIBRARY ${Python2_LIBRARIES}
            CACHE FILEPATH "Python library")
        set(PYTHON_INCLUDE_DIR ${Python2_INCLUDE_DIRS}
            CACHE PATH "Location of Python include files")
        set(PYTHON_NUMPY_INCLUDE_DIR ${Python2_NumPy_INCLUDE_DIRS}
            CACHE PATH "Location of NumPy include files.")
    else()
        set(PYTHON_LIBRARY ${Python3_LIBRARIES}
            CACHE FILEPATH "Python library")
        set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS}
            CACHE PATH "Location of Python include files.")
        set(PYTHON_NUMPY_INCLUDE_DIR ${Python3_NumPy_INCLUDE_DIRS}
            CACHE PATH "Location of NumPy include files.")
    endif()
    set(PDAL_HAVE_PYTHON 1)
elseif (NOT (CMAKE_VERSION VERSION_LESS "3.12.0"))
    find_package(Python3 COMPONENTS Interpreter Development)
    if (NOT Python3_FOUND)
        find_package(Python2 2.7 REQUIRED EXACT
            COMPONENTS Interpreter Development)

        # Since we've required 2.7, these should all be valid
        set(PYTHON_LIBRARY ${Python2_LIBRARIES}
            CACHE FILEPATH "Python library")
        set(PYTHON_INCLUDE_DIR ${Python2_INCLUDE_DIRS}
            CACHE PATH "Location of Python include files")
        set(PYTHON_EXECUTABLE ${Python2_EXECUTABLE})
        find_package(NumPy 1.5 REQUIRED)
    else()
        set(PYTHON_LIBRARY ${Python3_LIBRARIES}
	        CACHE FILEPATH "Python library")
        set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS}
	        CACHE PATH "Location of Python include files.")
         set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
         find_package(NumPy 1.5 REQUIRED)
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
set(PYTHON_ALL_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR} ${PYTHON_NUMPY_INCLUDE_DIR})
