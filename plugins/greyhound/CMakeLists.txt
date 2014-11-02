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
    add_definitions(-DHAVE_WEBSOCKETPP=1)

    #
    # Greyhound Reader
    #
    set(srcs
	drivers/GreyhoundReader.cpp
	drivers/WebSocketClient.cpp
    )

    set(incs
	drivers/Exchanges.hpp
	drivers/GreyhoundReader.hpp
	drivers/WebSocketClient.hpp
    )

    set(deps ${JSONCPP_LIBRARY})
    PDAL_ADD_PLUGIN(greyhound_reader_lib_name reader greyhound "${srcs}" "${incs}" "${deps}")
else()
    message(STATUS "Building without Greyhound support")
endif()
