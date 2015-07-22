#
# cmake policies
#
if (CMAKE_MAJOR_VERSION GREATER 2)
    cmake_policy(SET CMP0022 OLD) # interface link libraries
    cmake_policy(SET CMP0042 NEW) # osx rpath
endif()
