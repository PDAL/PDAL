#
# Matlab
#
find_package(Matlab QUIET)
set_package_properties(Matlab PROPERTIES TYPE REQUIRED)
if(MATLAB_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES "${MATLAB_LIBRARIES}")
    include_directories(SYSTEM ${MATLAB_INCLUDE_DIR})
    add_definitions(-DHAVE_MATLAB=1)
endif()
