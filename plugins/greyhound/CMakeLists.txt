#
# Greyhound plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin/greyhound)

find_package(JSONCPP REQUIRED)
find_package(WEBSOCKETPP REQUIRED)
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

    set(GREYHOUND_LIB_NAME pdal_plugin_reader_greyhound)
    PDAL_ADD_PLUGIN(${GREYHOUND_LIB_NAME} reader greyhound ${srcs} ${incs})
    target_link_libraries(${GREYHOUND_LIB_NAME} ${PDAL_LINKAGE} ${JSONCPP_LIBRARY})
else()
    message(STATUS "Building without Greyhound support")
endif()
