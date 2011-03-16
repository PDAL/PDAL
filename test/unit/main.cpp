/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <string>
#include <boost/test/unit_test.hpp>
#include <boost/test/utils/assign_op.hpp>

std::string g_data_path;

// Testing macros:
//   BOOST_TEST_MESSAGE("...")
//   BOOST_CHECK(bool)

//
// You can run the unit tests with these interesting options:
//
//     --log_format=X (-f X)   # X = xml|hrf
//     --log_level=X (-l X)    # X = error|message|all|... (default=error)
//     --log_sink=X (-k X)     # X = filename
//     
//     --report_format=X (-o X)
//     --report_level=X (-r X)
//     --report_sink=X (-e X)
//
//     --detect_memory_leaks=X # X = 0|1  (default=1)
//
//     <path>                  # path to data (default=../test/data)

#ifdef BOOST_TEST_ALTERNATIVE_INIT_API
bool
libpc_init_unit_test()
#else
::boost::unit_test::test_suite*
libpc_init_unit_test_suite( int, char* [] )
#endif
{
    ::boost::unit_test::assign_op( ::boost::unit_test::framework::master_test_suite().p_name.value, "Main", 0 );
    int argc = ::boost::unit_test::framework::master_test_suite().argc;
    char **argv = ::boost::unit_test::framework::master_test_suite().argv;
    if (argc > 1)
        g_data_path = argv[1];
    else
        g_data_path = "../test/data";
    if (g_data_path[g_data_path.size()] != '/')
        g_data_path += "/";
    
#ifdef BOOST_TEST_ALTERNATIVE_INIT_API
    return true;
#else
    return 0;
#endif
}

#if defined(BOOST_TEST_DYN_LINK)

int
main( int argc, char* argv[] )
{
    return ::boost::unit_test::unit_test_main( &libpc_init_unit_test, argc, argv );
}

#endif // BOOST_TEST_DYN_LINK
