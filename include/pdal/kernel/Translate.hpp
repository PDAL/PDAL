/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include "Application.hpp"

namespace pdal
{
namespace kernel
{

class PDAL_DLL Translate : public Application
{
public:
    Translate(int argc, const char* argv[]);
    int execute();

private:
    void addSwitches();
    void validateSwitches();
    
    Stage* makeReader(Options readerOptions);
    void forwardMetadata(Options & options, Metadata metadata);

    std::string m_inputFile;
    std::string m_outputFile;
    bool m_bCompress;
    boost::uint64_t m_numPointsToWrite; 
    boost::uint64_t m_numSkipPoints;
    pdal::SpatialReference m_input_srs;
    pdal::SpatialReference m_output_srs;
    pdal::Bounds<double> m_bounds;
    std::string m_wkt;
    std::string m_scales;
    std::string m_offsets;
    bool m_bForwardMetadata;
    boost::uint32_t m_decimation_step;
    boost::uint32_t m_decimation_offset;

};

} // namespace kernel
} // namespace pdal

