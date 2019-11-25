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

#include <pdal/Reader.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

void Reader::readerAddArgs(ProgramArgs& args)
{
    m_filenameArg = &args.add("filename", "Name of file to read", m_filename);
    m_countArg = &args.add("count", "Maximum number of points read", m_count,
        (std::numeric_limits<point_count_t>::max)());

    args.add("override_srs", "Spatial reference to apply to data",
            m_overrideSrs);
    args.addSynonym("override_srs", "spatialreference");

    args.add("default_srs",
            "Spatial reference to apply to data if one cannot be inferred",
            m_defaultSrs);
}


void Reader::setSpatialReference(MetadataNode& m, const SpatialReference& srs)
{
    if (srs.empty() && !m_defaultSrs.empty())
    {
        // If an attempt comes in to clear the SRS but we have a default,
        // revert to the default rather than clearing.
        Stage::setSpatialReference(m, m_defaultSrs);
        return;
    }

    if (getSpatialReference().empty() || m_overrideSrs.empty())
    {
        Stage::setSpatialReference(m, srs);
    }
    else
    {
        log()->get(LogLevel::Debug) <<
            "Ignoring setSpatialReference attempt: an override was set";
    }
}


void Reader::readerInitialize(PointTableRef)
{
    if (m_overrideSrs.valid() && m_defaultSrs.valid())
        throwError("Cannot specify both 'override_srs' and 'default_srs'");

    if (m_overrideSrs.valid())
        setSpatialReference(m_overrideSrs);
    else if (m_defaultSrs.valid())
        setSpatialReference(m_defaultSrs);
}

} // namespace pdal

