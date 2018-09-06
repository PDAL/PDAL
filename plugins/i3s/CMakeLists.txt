
add_definitions(-DHAVE_JSONCPP=1)
if (WIN32)
add_definitions("-DARBITER_DLL_IMPORT")
endif()

add_subdirectory(lepcc)  
set_property(TARGET lepcc PROPERTY POSITION_INDEPENDENT_CODE TRUE)


PDAL_ADD_PLUGIN(reader_libname reader i3s
    FILES
      io/i3sReader.cpp
      io/i3sReceiver.cpp
    LINK_WITH
        ${PDAL_JSONCPP_LIB_NAME}
        ${WINSOCK_LIBRARY}
)
target_link_libraries(${reader_libname} PRIVATE
    lepcc
)

target_include_directories(${reader_libname} PRIVATE
    ${PDAL_JSONCPP_INCLUDE_DIR}
    ${PDAL_VENDOR_DIR}
    ${LEPCC_INCLUDE_DIR}
)
