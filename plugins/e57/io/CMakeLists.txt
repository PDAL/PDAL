add_library(pdal_plugin_reader_e57 SHARED e57reader.cpp utils.cpp scan.cpp uuid.cpp)
target_link_libraries(pdal_plugin_reader_e57 E57Format ${XercesC_LIBRARY} ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_reader_e57 PRIVATE ${libe57format_INCLUDE} "${spdlog_SOURCE_DIR}/include")
target_include_directories(pdal_plugin_reader_e57 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(pdal_plugin_writer_e57 SHARED e57writer.cpp utils.cpp scan.cpp uuid.cpp)
target_link_libraries(pdal_plugin_writer_e57 E57Format ${XercesC_LIBRARY} ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_writer_e57 PRIVATE ${libe57format_INCLUDE} "${spdlog_SOURCE_DIR}/include")
target_include_directories(pdal_plugin_writer_e57 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

#------------------------------------------------------------------------------
# Install
#------------------------------------------------------------------------------
INSTALL(TARGETS pdal_plugin_reader_e57 pdal_plugin_writer_e57
        LIBRARY DESTINATION lib
        COMPONENT pdal_e57io)