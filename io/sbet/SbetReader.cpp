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

#include <pdal/PointRef.hpp>

#include "SbetReader.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.sbet",
    "SBET Reader",
    "http://pdal.io/stages/readers.sbet.html" );

CREATE_STATIC_PLUGIN(1, 0, SbetReader, Reader, s_info)

std::string SbetReader::getName() const { return s_info.name; }

Options SbetReader::getDefaultOptions()
{
    Options options;
    return options;
}


void SbetReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(getDefaultDimensions());
}


void SbetReader::ready(PointTableRef)
{
    size_t fileSize = FileUtils::fileSize(m_filename);
    size_t pointSize = getDefaultDimensions().size() * sizeof(double);
    if (fileSize % pointSize != 0)
        throw pdal_error("invalid sbet file size");
    m_numPts = fileSize / pointSize;
    m_index = 0;
    m_stream.reset(new ILeStream(m_filename));
    m_dims = fileDimensions();
    seek(m_index);
}


bool SbetReader::processOne(PointRef& point)
{
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        double d;
        *m_stream >> d;
        Dimension::Id::Enum dim = *di;
        point.setField(dim, d);
    }
    return (m_stream->good());
}


point_count_t SbetReader::read(PointViewPtr view, point_count_t count)
{
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;
    seek(idx);
    while (numRead < count && idx < m_numPts)
    {
        PointRef point = view->point(nextId);
        processOne(point);
        if (m_cb)
            m_cb(*view, nextId);

        idx++;
        nextId++;
        numRead++;
    }
    m_index = idx;
    return numRead;
}


bool SbetReader::eof()
{
    return m_index >= m_numPts;
}


void SbetReader::seek(PointId idx)
{
    m_stream->seek(idx * sizeof(double) * getDefaultDimensions().size());
}

} // namespace pdal
