/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include "SbetWriter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.sbet",
    "SBET Writer",
    "http://pdal.io/stages/writers.sbet.html",
    { "sbet" }
};

CREATE_STATIC_STAGE(SbetWriter, s_info)

std::string SbetWriter::getName() const { return s_info.name; }

void SbetWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("angles_are_degrees", "Angles coming into the writer are in degrees", m_anglesAreDegrees, true);
}


void SbetWriter::ready(PointTableRef)
{
    m_stream.reset(new OLeStream(m_filename));
}


void SbetWriter::write(const PointViewPtr view)
{
    auto degreesToRadians = [](double degrees) {
        return degrees * M_PI / 180.0;
    };
    Dimension::IdList dims = sbet::fileDimensions();
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            // If a dimension doesn't exist, write 0.
            Dimension::Id dim = *di;
            double value = (view->hasDim(dim) ? view->getFieldAs<double>(dim, idx) : 0.0);
            if (m_anglesAreDegrees && sbet::isAngularDimension(dim)) {
                value = degreesToRadians(value);
            }
            *m_stream << value;
        }
    }
}


void SbetWriter::done(PointTableRef table)
{
    getMetadata().addList("filename", m_filename);
}

} // namespace pdal
