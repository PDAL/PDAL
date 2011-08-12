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

#ifndef UNITTEST_SUPPORT_INCLUDED
#define UNITTEST_SUPPORT_INCLUDED

// support functions for unit testing

#include <pdal/Bounds.hpp>
#include <boost/cstdint.hpp>

namespace pdal
{
    class PointBuffer;
    class Schema;
}

#include <boost/cstdint.hpp>
#include <string>

class Support
{
public:
    // this is where the reference files (input data) live
    static std::string datapath();

    // returns "datapath + / + file"
    static std::string datapath(const std::string& file);

    // this is where the temporary output files go
    static std::string temppath();

    // returns "temppath + / + file"
    static std::string temppath(const std::string& file);

    // this is where the pdal executables live
    static std::string binpath();

    // return "bindir + / + file"
    static std::string binpath(const std::string& file);

    // returns "name" on unix and "name + .exe" on windows
    static std::string exename(const std::string& name);

    // returns number of bytes different for two binary files (or maxint if a file doesn't exist)
    static boost::uint32_t diff_files(const std::string& file1, const std::string& file2);

    // same as diff_files, but allows for regions of the file be to be ignored
    //
    // ignorable_start/length are arrays of byte-offsets and byte-lengths,
    // for regions in the file we wish to NOT do the comparison on.  (We are 
    // assuming such an ignorable region exists with the same length in both 
    // files, such as would be the case if an embedded version number in two
    // LAS files was different.)  The number of ignorable regions is set
    // via num_ignorables.
    static boost::uint32_t diff_files(const std::string& file1, const std::string& file2,
                                      boost::uint32_t* ignorable_start, boost::uint32_t* ignorable_length, boost::uint32_t num_ignorables);

    // returns number of lines different for two text files (or maxint if a file doesn't exist)
    static boost::uint32_t diff_text_files(const std::string& file1, const std::string& file2);

    // returns true iff the two (binary or ascii) files are the same,
    // using the above diff_files/diff_text_files functions
    static bool compare_files(const std::string& file1, const std::string& file2);
    static bool compare_text_files(const std::string& file1, const std::string& file2);

    // validate a point's XYZ values
    static void check_pN(const pdal::PointBuffer& data, const pdal::Schema& schema, 
                         std::size_t index, 
                         double xref, double yref, double zref);
                       
    // validate a point's XYZ, Time, and Color values
    static void check_pN(const pdal::PointBuffer& data, const pdal::Schema& schema, 
                         std::size_t index, 
                         double xref, double yref, double zref,
                         double tref,
                         boost::uint16_t rref, boost::uint16_t gref, boost::uint16_t bref);

    // these are for the 1.2-with-color image
    static void check_p0_p1_p2(const pdal::PointBuffer& data, const pdal::Schema& schema);
    static void check_p100_p101_p102(const pdal::PointBuffer& data, const pdal::Schema& schema);
    static void check_p355_p356_p357(const pdal::PointBuffer& data, const pdal::Schema& schema);
    static void check_p710_p711_p712(const pdal::PointBuffer& data, const pdal::Schema& schema);

    static void compareBounds(const pdal::Bounds<double>& p, const pdal::Bounds<double>& q);

    // executes "cmd" via popen, copying stdout into output and returning the status code
    //
    // note: under windows, all "/" characrters in cmd will be converted to "\\" for you
    static int run_command(const std::string& cmd, std::string& output);
};


#endif
