/******************************************************************************
* Copyright (c) 2014, Andrew Bell
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

#include <pdal/drivers/bpf/BpfReader.hpp>
#include <pdal/drivers/bpf/BpfSeqIterator.hpp>
#include <pdal/Options.hpp>

const std::string utm_wkt1 =
    "PROJCS[\"WGS 84 / UTM zone ";

const std::string utm_wkt2 =
    "\",\
    GEOGCS[\"WGS 84\",\
        DATUM[\"WGS_1984\",\
            SPHEROID[\"WGS 84\",6378137,298.257223563,\
                AUTHORITY[\"EPSG\",\"7030\"]],\
            AUTHORITY[\"EPSG\",\"6326\"]],\
        PRIMEM[\"Greenwich\",0,\
            AUTHORITY[\"EPSG\",\"8901\"]],\
        UNIT[\"degree\",0.01745329251994328,\
            AUTHORITY[\"EPSG\",\"9122\"]],\
        AUTHORITY[\"EPSG\",\"4326\"]],\
    UNIT[\"metre\",1,\
        AUTHORITY[\"EPSG\",\"9001\"]],\
    PROJECTION[\"Transverse_Mercator\"],\
    PARAMETER[\"latitude_of_origin\",0],\
    PARAMETER[\"central_meridian\",-57],\
    PARAMETER[\"scale_factor\",0.9996],\
    PARAMETER[\"false_easting\",500000],\
    PARAMETER[\"false_northing\",10000000],\
    AUTHORITY[\"EPSG\",\"32721\"],\
    AXIS[\"Easting\",EAST],\
    AXIS[\"Northing\",NORTH]]";

namespace pdal
{

BpfReader::BpfReader(const Options& options) : Reader(options),
    m_stream(options.getValueOrThrow<std::string>("filename"))
{}


BpfReader::BpfReader(const std::string& filename) : m_stream(filename)
{}


// When the stage is intialized, the schema needs to be populated with the
// dimensions in order to allow subsequent stages to be aware of or append to
// the dimensions in the PointBuffer.
void BpfReader::initialize()
{
    // In order to know the dimensions we must read the file header.
    if (!m_header.read(m_stream))
        return;

    m_dims.insert(m_dims.end(), m_header.m_numDim, BpfDimension());
    if (!BpfDimension::read(m_stream, m_dims))
        return;

    readUlemData();
    if (!m_stream)
        return;
    readPolarData();

    std::string wkt = utm_wkt1 +
    boost::lexical_cast<std::string>(abs(m_header.m_coordId)) +
    (m_header.m_coordId > 0 ? "N" : "S") +
    utm_wkt2;
    SpatialReference srs;
    srs.setWKT(wkt);
    setSpatialReference(srs);

    // Fast forward file to end of header as reported by base header.
    std::streampos pos = m_stream.position();
    if (pos > m_header.m_len)
        throw "BPF Header length exceeded that reported by file.";
    else if (pos < m_header.m_len)
        m_stream.seek(m_header.m_len);
}

void BpfReader::buildSchema(Schema *schema)
{
    for (size_t i = 0; i < m_dims.size(); ++i)
    {
        BpfDimension& dim = m_dims[i];
        Dimension pd(dim.m_label, dimension::Float, sizeof(float));
        pd.setMinimum(dim.m_min);
        pd.setMaximum(dim.m_max);
        pd.setNumericOffset(dim.m_offset);
        pd.setNamespace("bpf");
        m_schemaDims.push_back(schema->appendDimension(pd));
    }
}

bool BpfReader::readUlemData()
{
    if (!m_ulemHeader.read(m_stream))
        return false;

    for (size_t i = 0; i < m_ulemHeader.m_numFrames; i++)
    {
        BpfUlemFrame frame;
        if (!frame.read(m_stream))
            return false;
        m_ulemFrames.push_back(frame);
    }

    BpfUlemFile file;
    while (file.read(m_stream))
        ;

    return (bool)m_stream;
}

bool BpfReader::readPolarData()
{
    if (!m_polarHeader.read(m_stream))
        return false;
    for (size_t i = 0; i < m_polarHeader.m_numFrames; ++i)
    {
        BpfPolarFrame frame;
        if (!frame.read(m_stream))
            return false;
        m_polarFrames.push_back(frame);
    }
    return (bool)m_stream;
}

StageSequentialIterator *BpfReader::createSequentialIterator() const
{
    return new BpfSeqIterator(m_schemaDims, m_header.m_numPts,
        m_header.m_pointFormat, m_header.m_compression,
        const_cast<ILeStream&>(m_stream));
}

StageRandomIterator *
BpfReader::createRandomIterator(PointBuffer& pb) const
{
    return NULL;
}

} //namespace
