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

#include "SbetReader.hpp"
#include "SbetCommon.hpp"

#include "private/connector/Connector.hpp"

#include <pdal/PointRef.hpp>
#include <pdal/util/FileUtils.hpp>

#include <cmath>

namespace pdal
{

struct SbetReader::Private
{
    std::unique_ptr<ILeStream> stream;
    // Number of points in the file.
    point_count_t numPts;
    point_count_t index;
    Dimension::IdList dims;
    bool anglesAsDegrees;
    
    std::unique_ptr<connector::Connector> connector;
    bool isRemote = false;
};

SbetReader::SbetReader() : m_private(new Private)
{}

SbetReader::~SbetReader()
{ 
    cleanup(); 
}

static StaticPluginInfo const s_info
{
    "readers.sbet",
    "SBET Reader",
    "https://pdal.org/stages/readers.sbet.html",
    { "sbet" }
};

CREATE_STATIC_STAGE(SbetReader, s_info)

std::string SbetReader::getName() const { return s_info.name; }

void SbetReader::addArgs(ProgramArgs& args)
{
    args.add("angles_as_degrees", "Convert all angles to degrees", m_private->anglesAsDegrees, true);
}

void SbetReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(sbet::fileDimensions());
}


void SbetReader::ready(PointTableRef)
{
    tryLoadRemote();

    size_t fileSize = FileUtils::fileSize(m_filename);
    size_t pointSize = sbet::fileDimensions().size() * sizeof(double);
    if ((fileSize == 0)|| (fileSize % pointSize != 0))
        throwError("Invalid file size.");
    m_private->numPts = fileSize / pointSize;
    m_private->index = 0;
    m_private->stream.reset(new ILeStream(m_filename));
    m_private->dims = sbet::fileDimensions();
    seek(m_private->index);
}


bool SbetReader::processOne(PointRef& point)
{
    auto radiansToDegrees = [](double radians) {
        return radians * 180.0 / M_PI;
    };
    for (auto di = m_private->dims.begin(); di != m_private->dims.end(); ++di)
    {
        double d;
        *(m_private->stream) >> d;
        Dimension::Id dim = *di;
        if (m_private->anglesAsDegrees && sbet::isAngularDimension(dim)) {
            d = radiansToDegrees(d);
        }
        point.setField(dim, d);
    }
    return (m_private->stream->good());
}


point_count_t SbetReader::read(PointViewPtr view, point_count_t count)
{
    PointId nextId = view->size();
    PointId idx = m_private->index;
    point_count_t numRead = 0;
    seek(idx);
    while (numRead < count && idx < m_private->numPts)
    {
        PointRef point = view->point(nextId);
        processOne(point);
        if (m_cb)
            m_cb(*view, nextId);

        idx++;
        nextId++;
        numRead++;
    }
    m_private->index = idx;
    return numRead;
}


bool SbetReader::eof()
{
    return m_private->index >= m_private->numPts;
}


void SbetReader::seek(PointId idx)
{
    m_private->stream->seek(idx * sizeof(double) * sbet::fileDimensions().size());
}

void SbetReader::done(PointTableRef table)
{
    cleanup();
}

void SbetReader::cleanup()
{
    if (m_private->isRemote)
        FileUtils::deleteFile(m_filename);
}

void SbetReader::tryLoadRemote()
{
    m_private->connector.reset(new connector::Connector(m_filespec));
    m_private->isRemote = Utils::isRemote(m_filename);
    auto handle = m_private->connector->getLocalHandle(m_filename);
    m_filename = handle.release();
}

} // namespace pdal
