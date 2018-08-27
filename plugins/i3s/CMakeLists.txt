
PDAL_ADD_PLUGIN(reader_libname reader i3s
    FILES
    io/i3sReader.cpp
    LINK_WITH
        ${PDAL_JSONCPP_LIB_NAME}
        ${WINSOCK_LIBRARY}
)


