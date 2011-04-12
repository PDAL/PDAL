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

#include "support.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>

#include <libpc/Utils.hpp>


bool compare_files(const std::string& file1, const std::string& file2)
{
    uintmax_t len1x = libpc::Utils::fileSize(file1);
    uintmax_t len2x = libpc::Utils::fileSize(file2);
    size_t len1 = (size_t)len1x; // BUG
    size_t len2 = (size_t)len2x;

    if (len1 != len2)
    {
        return false;
    }

    std::istream* str1 = libpc::Utils::openFile(file1);
    std::istream* str2 = libpc::Utils::openFile(file2);
    BOOST_CHECK(str1);
    BOOST_CHECK(str2);

    char* buf1 = new char[len1];
    char* buf2 = new char[len2];

    str1->read(buf1,len1);
    str2->read(buf2,len2);

    libpc::Utils::closeFile(str1);
    libpc::Utils::closeFile(str2);

    char* p = buf1;
    char* q = buf2;

    for (uintmax_t i=0; i<len1; i++)
    {
        if (*p != *q) 
        {
            delete[] buf1;
            delete[] buf2;
            return false;
        }
        ++p;
        ++q;
    }

    delete[] buf1;
    delete[] buf2;

    return true;
}

std::string TestConfig::g_data_path = "../../test/data";
std::string TestConfig::g_oracle_connection = "lidar/lidar@oracle.hobu.biz/orcl";
TestConfig::TestConfig()
{
    int argc = ::boost::unit_test::framework::master_test_suite().argc;
    char **argv = ::boost::unit_test::framework::master_test_suite().argv;
    if (argc > 1)
    {
        g_data_path = argv[1];
        if (argc > 2)
            g_oracle_connection = argv[2];        
    }
    if (g_data_path[g_data_path.size() - 1] != '/')
        g_data_path += "/";
}

