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
#include <pdal/pdal.hpp>

namespace pdal
{
    class PointBuffer;
    class Schema;
    class Stage;
}

#include <boost/cstdint.hpp>
#include <string>

class Support
{
public:
    static std::string datapath(const std::string&);

    static bool compare_stage_data(pdal::Stage const& a, pdal::Stage const& b);
    
    // verify if two files are the same
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
};


#endif
