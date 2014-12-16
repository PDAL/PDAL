#
# Greyhound plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/greyhound)

find_package(JSONCPP)
find_package(WEBSOCKETPP)
if(JSONCPP_FOUND AND WEBSOCKETPP_FOUND)
    message(STATUS "Building with Greyhound support")

    # add JSONCPP stuff
    include_directories(${JSONCPP_INCLUDE_DIR})
    add_definitions(-DHAVE_JSONCPP=1)

    # add WEBSOCKETPP stuff
    include_directories(${WEBSOCKETPP_INCLUDE_DIR})
    add_definitions(-D_WEBSOCKETPP_NULLPTR_TOKEN_=0)
    add_definitions(-D_WEBSOCKETPP_CPP11_RANDOM_DEVICE_=1)
    add_definitions(-DHAVE_WEBSOCKETPP=1)

    set(srcs
        io/GreyhoundReader.cpp
        io/WebSocketClient.cpp
    )

    set(incs
        io/Exchanges.hpp
        io/GreyhoundReader.hpp
        io/WebSocketClient.hpp
    )

    PDAL_ADD_PLUGIN(libname reader greyhound
        FILES "${srcs}" "${incs}"
        LINK_WITH ${JSONCPP_LIBRARY})
else()
    message(STATUS "Building without Greyhound support")
endif()
