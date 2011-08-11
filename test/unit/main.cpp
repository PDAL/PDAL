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

#define BOOST_TEST_MODULE Main
#include <boost/test/unit_test.hpp>

#include "TestConfig.hpp"

BOOST_GLOBAL_FIXTURE(TestConfig)

// Testing macros:
//   BOOST_TEST_MESSAGE("...")
//   BOOST_CHECK(bool)
//   BOOST_CHECK_EQUAL(t1, t2)
//
// The 'CHECK_EQUAL function is preferred to plain 'CHECK, because the former
// prints out the values of both arguments in the error message.
//
// For comparing to floating point values, use
//   BOOST_CHECK_CLOSE(a,b,perc)
// where perc is a percentage value in the range [0..100] (typically)

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

