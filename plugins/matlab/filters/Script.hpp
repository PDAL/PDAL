/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#include <pdal/pdal_internal.hpp>

#include <matrix.h>

#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Log.hpp>

namespace pdal
{
namespace mlang
{

class PDAL_DLL Script
{
public:
    Script() {};

    static int getMatlabDataType(Dimension::Type t);
    static Dimension::Type getPDALDataType(mxClassID t);

    static mxArray* setMatlabStruct(PointViewPtr view, const Dimension::IdList& dims, const std::string& pdalargs, MetadataNode node, LogPtr log);

    static void getMatlabStruct(mxArray* array, PointViewPtr view, const Dimension::IdList& dims, std::string& pdalargs, MetadataNode node, LogPtr log);
    static PointLayoutPtr getStructLayout(mxArray* array, LogPtr log);
    static std::string getLogicalMask(mxArray* array, LogPtr log);
    static std::string getSRSWKT(mxArray* array, LogPtr log);

    std::string m_source;
    std::string m_scriptFilename;

private:

    Script& operator=(Script const& rhs); // nope
};

PDAL_DLL std::ostream& operator<<(std::ostream& os, Script const& d);

} // namespace mlang
} // namespace pdal

