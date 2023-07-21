/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)*
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
*
****************************************************************************/

#include "ArrowReader.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/PDALUtils.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.arrow",
    "Arrow Reader",
    "http://pdal.io/stages/readers.arrow.html"
};

CREATE_SHARED_STAGE(ArrowReader, s_info)

std::string ArrowReader::getName() const { return s_info.name; }


void ArrowReader::addArgs(ProgramArgs& args)
{
//     args.add("rdtp", "", m_isRdtp, DEFAULT_IS_RDTP);
}

void ArrowReader::initialize()
{

    if (pdal::Utils::isRemote(m_filename))
        m_filename = pdal::Utils::fetchRemote(m_filename);

}


void ArrowReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    Dimension::IdList ids;

    ids.push_back(Id::X);
    ids.push_back(Id::Y);
    ids.push_back(Id::Z);
    layout->registerDims(ids);
}


void ArrowReader::ready(PointTableRef table)
{
}


point_count_t ArrowReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t numRead = 0;
    PointRef point(view->point(0));
    while (numRead < num && true /*not at end of arrow array*/ ) {
        point.setPointId(numRead);
        processOne(point);
        ++numRead;
    }
    return numRead;
}


bool ArrowReader::processOne(PointRef& point)
{
    return true;
}


void ArrowReader::done(PointTableRef table)
{
}


} // namespace pdal
