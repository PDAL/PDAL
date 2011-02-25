//BUG FIXME: this should be driven by cmake to determine if we're using
#ifdef _MSC_VER
#define BOOST_TEST_DYN_LINK
#endif

#define BOOST_TEST_MODULE Main
#include <boost/test/unit_test.hpp>


/* actually no code in here, this is just the main file */

// Testing macros:
//   BOOST_TEST_MESSAGE("...")
//   BOOST_CHECK(bool)


//
// you can run the unit tests as "bin/../libpc_test" with these interesting options:
//
//     --log_format=X (-f X)     # where X is xml|hrf
//
//     --log_level=X (-l X)      # where X is error|message|all|...    (default is error)
//
//     --log_sink=X (-k X)       # where X is filename
//     
//     --report_format=X (-o X)
//     --report_level=X (-r X)
//     --report_sink=X (-e X)
//
//     --detect_memory_leaks=X    # where X is 0|1  (default is 1)
