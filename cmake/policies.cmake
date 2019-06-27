#
# cmake policies
#
if (CMAKE_MAJOR_VERSION GREATER 2)
    cmake_policy(SET CMP0022 NEW) # interface link libraries
    cmake_policy(SET CMP0042 NEW) # osx rpath
    if (CMAKE_MAJOR_VERSION GREATER 3 AND
        CMAKE_MINOR_VERSION GREATER 12)
        cmake_policy(SET CMP0075 NEW) # Mystical setting to eliminate warning
    endif()
endif()
