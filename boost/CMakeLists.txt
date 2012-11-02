# These defines are conditionally set in the Jamfiles but I did nothing to try
# to perpetuate them in this build:
#   DATE_TIME_INLINE
#   BOOST_THREAD_POSIX
#

include_directories(".")

if(WIN32)
    #add_definitions(-DBOOST_TEST_DYN_LINK)
    #add_definitions(-DBOOST_TEST_NO_MAIN)
	
	# disable compiler warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
	
else()
    
	add_definitions(-DBOOST_TEST_DYN_LINK)

endif()


set(PDALBOOST_SOURCES
    ./libs/date_time/src/gregorian/date_generators.cpp
    ./libs/date_time/src/gregorian/greg_month.cpp
    ./libs/date_time/src/gregorian/greg_weekday.cpp
    ./libs/date_time/src/gregorian/gregorian_types.cpp
    ./libs/date_time/src/posix_time/posix_time_types.cpp
    ./libs/exception/src/clone_current_exception_non_intrusive.cpp
    ./libs/filesystem
    ./libs/filesystem/src/windows_file_codecvt.hpp
    ./libs/filesystem/src/path_traits.cpp
    ./libs/filesystem/src/unique_path.cpp
    ./libs/filesystem/src/codecvt_error_category.cpp
    ./libs/filesystem/src/path.cpp
    ./libs/filesystem/src/operations.cpp
    ./libs/filesystem/src/utf8_codecvt_facet.cpp
    ./libs/iostreams/src/file_descriptor.cpp
    ./libs/iostreams/src/mapped_file.cpp
    ./libs/program_options/src/cmdline.cpp
    ./libs/program_options/src/config_file.cpp
    ./libs/program_options/src/convert.cpp
    ./libs/program_options/src/options_description.cpp
    ./libs/program_options/src/parsers.cpp
    ./libs/program_options/src/positional_options.cpp
    ./libs/program_options/src/split.cpp
    ./libs/program_options/src/utf8_codecvt_facet.cpp
    ./libs/program_options/src/value_semantic.cpp
    ./libs/program_options/src/variables_map.cpp
    ./libs/program_options/src/winmain.cpp
    ./libs/serialization/src/archive_exception.cpp
    ./libs/serialization/src/basic_archive.cpp
    ./libs/serialization/src/basic_iarchive.cpp
    ./libs/serialization/src/basic_iserializer.cpp
    ./libs/serialization/src/basic_oarchive.cpp
    ./libs/serialization/src/basic_oserializer.cpp
    ./libs/serialization/src/basic_pointer_iserializer.cpp
    ./libs/serialization/src/basic_pointer_oserializer.cpp
    ./libs/serialization/src/basic_serializer_map.cpp
    ./libs/serialization/src/basic_text_iprimitive.cpp
    ./libs/serialization/src/basic_text_oprimitive.cpp
    ./libs/serialization/src/basic_text_wiprimitive.cpp
    ./libs/serialization/src/basic_text_woprimitive.cpp
    ./libs/serialization/src/basic_xml_archive.cpp
    ./libs/serialization/src/binary_iarchive.cpp
    ./libs/serialization/src/binary_oarchive.cpp
    ./libs/serialization/src/binary_wiarchive.cpp
    ./libs/serialization/src/binary_woarchive.cpp
    ./libs/serialization/src/codecvt_null.cpp
    ./libs/serialization/src/extended_type_info.cpp
    ./libs/serialization/src/extended_type_info_no_rtti.cpp
    ./libs/serialization/src/extended_type_info_typeid.cpp
    ./libs/serialization/src/polymorphic_iarchive.cpp
    ./libs/serialization/src/polymorphic_oarchive.cpp
    ./libs/serialization/src/shared_ptr_helper.cpp
    ./libs/serialization/src/stl_port.cpp
    ./libs/serialization/src/text_iarchive.cpp
    ./libs/serialization/src/text_oarchive.cpp
    ./libs/serialization/src/text_wiarchive.cpp
    ./libs/serialization/src/text_woarchive.cpp
    ./libs/serialization/src/utf8_codecvt_facet.cpp
    ./libs/serialization/src/void_cast.cpp
    ./libs/serialization/src/xml_archive_exception.cpp
    ./libs/serialization/src/xml_grammar.cpp
    ./libs/serialization/src/xml_iarchive.cpp
    ./libs/serialization/src/xml_oarchive.cpp
    ./libs/serialization/src/xml_wgrammar.cpp
    ./libs/serialization/src/xml_wiarchive.cpp
    ./libs/system/src/error_code.cpp
    ./libs/test/src/compiler_log_formatter.cpp
    ./libs/test/src/cpp_main.cpp
    ./libs/test/src/debug.cpp
    ./libs/test/src/exception_safety.cpp
    ./libs/test/src/execution_monitor.cpp
    ./libs/test/src/framework.cpp
    ./libs/test/src/interaction_based.cpp
    ./libs/test/src/logged_expectations.cpp
    ./libs/test/src/plain_report_formatter.cpp
    ./libs/test/src/progress_monitor.cpp
    ./libs/test/src/results_collector.cpp
    ./libs/test/src/results_reporter.cpp
    ./libs/test/src/test_main.cpp
    ./libs/test/src/test_tools.cpp
    ./libs/test/src/unit_test_log.cpp
    ./libs/test/src/unit_test_main.cpp
    ./libs/test/src/unit_test_monitor.cpp
    ./libs/test/src/unit_test_parameters.cpp
    ./libs/test/src/unit_test_suite.cpp
    ./libs/test/src/xml_log_formatter.cpp
    ./libs/test/src/xml_report_formatter.cpp
)
if(WIN32)
  list(APPEND 
       ./libs/thread/src/win32/thread.cpp
       ./libs/filesystem/src/windows_file_codecvt.hpp
       ./libs/filesystem/src/windows_file_codecvt.cpp
       ./libs/thread/src/win32/tss_dll.cpp
       ./libs/thread/src/win32/tss_pe.cpp)
else()
  list(APPEND PDALBOOST_SOURCES
       ./libs/thread/src/pthread/once.cpp
       ./libs/thread/src/pthread/thread.cpp)
endif(WIN32)

add_library(${PDALBOOST_LIB_NAME} STATIC ${PDALBOOST_SOURCES})

