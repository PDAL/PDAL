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

#pragma once

// support functions for unit testing

#include <pdal/pdal_types.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{
    class PointView;
    class Stage;
}

#include <string>

class Support
{
public:
    // this is where the reference files (input data) live
    static std::string datapath();

    // returns "datapath + / + file"
    static std::string datapath(const std::string& file);

    // this is where the reference files (input data) live
    static std::string configuredpath();

    // returns "configuredpath + / + file"
    static std::string configuredpath(const std::string& file);


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

    // returns number of bytes different for two binary files (or maxint
    // if a file doesn't exist)
    static uint32_t diff_files(const std::string& file1,
       const std::string& file2);
    static uint32_t diff_files(std::istream& str1, std::istream& str2);

    // same as diff_files, but allows for a region of the file be to be ignored
    // (region is specified with a starting offset and a length)
    static uint32_t diff_files(const std::string& file1,
        const std::string& file2, uint32_t ignorable_start,
        uint32_t ignorable_length);

    // same as above diff_files with ignorable region, but for multiple regions
    static uint32_t diff_files(const std::string& file1,
        const std::string& file2, uint32_t* ignorable_start,
        uint32_t* ignorable_length, uint32_t num_ignorables);

    // same as above diff_files with ignorable region, but for multiple regions
    static uint32_t diff_files(std::istream& str1, std::istream& str2,
        uint32_t* ignorable_start, uint32_t* ignorable_length,
        uint32_t num_ignorables);

    // returns number of lines different for two text files (or maxint
    // if a file doesn't exist) if ignoreLine is not -1, that line will
    // be "ignored" when comparing the two files
    static uint32_t diff_text_files(const std::string& file1,
        const std::string& file2, int32_t ignoreLine1=-1);
    static uint32_t diff_text_files(std::istream& str1, std::istream& str2,
        int32_t ignoreLine1=-1);

    // returns true iff the two (binary or ascii) files are the same,
    // using the above diff_files/diff_text_files functions

    static bool compare_files(const std::string& file1,
        const std::string& file2);
    static bool compare_text_files(const std::string& file1,
        const std::string& file2);
    static bool compare_text_files(std::istream& str1, std::istream& str2);

    // validate a point's XYZ values
    static void check_pN(const pdal::PointView& data,
                         pdal::PointId index,
                         double xref, double yref, double zref);

    // validate a point's XYZ, Time, and Color values
    static void check_pN(const pdal::PointView& data,
        pdal::PointId index, double xref, double yref, double zref,
        double tref, uint16_t rref, uint16_t gref, uint16_t bref);

    // these are for the 1.2-with-color image
    static void check_p0_p1_p2(const pdal::PointView& data);
    static void check_p100_p101_p102(const pdal::PointView& data);
    static void check_p355_p356_p357(const pdal::PointView& data);
    static void check_p710_p711_p712(const pdal::PointView& data);

    static void compareBounds(const pdal::BOX3D& p,
        const pdal::BOX3D& q);

    // executes "cmd" via popen, copying stdout into output and returning
    // the status code note: under windows, all "/" characrters in cmd will
    // be converted to "\\" for you
    // static int run_command(const std::string& cmd, std::string& output);
};

