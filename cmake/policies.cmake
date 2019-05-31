#
# cmake policies
#
if (CMAKE_MAJOR_VERSION GREATER 2)
    cmake_policy(SET CMP0022 NEW) # interface link libraries
    cmake_policy(SET CMP0042 NEW) # osx rpath
    cmake_policy(SET CMP0075 NEW) # Mystical setting to eliminate warning
endif()
