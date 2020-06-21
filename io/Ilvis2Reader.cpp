/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include "Ilvis2Reader.hpp"
#include "Ilvis2MetadataReader.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <algorithm>
#include <cmath>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.ilvis2",
    "ILVIS2 Reader",
    "http://pdal.io/stages/readers.ilvis2.html"
};

CREATE_STATIC_STAGE(Ilvis2Reader, s_info)

std::string Ilvis2Reader::getName() const { return s_info.name; }

std::istream& operator >> (std::istream& in, Ilvis2Reader::IlvisMapping& mval)
{
    std::string s;

    in >> s;
    s = Utils::toupper(s);

    static std::map<std::string, Ilvis2Reader::IlvisMapping> m =
        { { "INVALID", Ilvis2Reader::IlvisMapping::INVALID },
          { "LOW", Ilvis2Reader::IlvisMapping::LOW },
          { "HIGH", Ilvis2Reader::IlvisMapping::HIGH },
          { "ALL", Ilvis2Reader::IlvisMapping::ALL } };

    mval = m[s];
    return in;
}


std::ostream& operator<<(std::ostream& out,
    const Ilvis2Reader::IlvisMapping& mval)
{
    switch (mval)
    {
    case Ilvis2Reader::IlvisMapping::INVALID:
        out << "Invalid";
    case Ilvis2Reader::IlvisMapping::LOW:
        out << "Low";
    case Ilvis2Reader::IlvisMapping::HIGH:
        out << "High";
    case Ilvis2Reader::IlvisMapping::ALL:
        out << "All";
    }
    return out;
}


Ilvis2Reader::Ilvis2Reader()
{}


Ilvis2Reader::~Ilvis2Reader()
{}


void Ilvis2Reader::addArgs(ProgramArgs& args)
{
    args.add("mapping", "Mapping for values", m_mapping, IlvisMapping::ALL);
    args.add("metadata", "Metadata file", m_metadataFile);
}


void Ilvis2Reader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::LvisLfid);
    layout->registerDim(Dimension::Id::ShotNumber);
    layout->registerDim(Dimension::Id::GpsTime);
    layout->registerDim(Dimension::Id::LongitudeCentroid);
    layout->registerDim(Dimension::Id::LatitudeCentroid);
    layout->registerDim(Dimension::Id::ElevationCentroid);
    layout->registerDim(Dimension::Id::LongitudeLow);
    layout->registerDim(Dimension::Id::LatitudeLow);
    layout->registerDim(Dimension::Id::ElevationLow);
    layout->registerDim(Dimension::Id::LongitudeHigh);
    layout->registerDim(Dimension::Id::LatitudeHigh);
    layout->registerDim(Dimension::Id::ElevationHigh);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
}


void Ilvis2Reader::initialize(PointTableRef)
{
    if (!m_metadataFile.empty() && !FileUtils::fileExists(m_metadataFile))
        throwError("Invalid metadata file: '" + m_metadataFile + "'");

    // Data are WGS84 (4326) with ITRF2000 datum (6656)
    // See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html for
    // background
    setSpatialReference("EPSG:4326");
}


template <typename T>
T convert(const StringList& s, const std::string& name, size_t fieldno)
{
    T output;
    if (!Utils::fromString(s[fieldno], output))
        throw Ilvis2Reader::error("Unable to convert " + name +
            ", " + s[fieldno] + ", to double");

    return output;
}


void Ilvis2Reader::readPoint(PointRef& point, StringList s,
    std::string pointMap)
{
    point.setField(pdal::Dimension::Id::LvisLfid,
        convert<unsigned>(s, "LVIS_LFID", 0));
    point.setField(pdal::Dimension::Id::ShotNumber,
        convert<unsigned>(s, "SHOTNUMBER", 1));
    point.setField(pdal::Dimension::Id::GpsTime,
        convert<double>(s, "GPSTIME", 2));
    point.setField(pdal::Dimension::Id::LongitudeCentroid,
        Utils::normalizeLongitude(convert<double>(s, "LONGITUDE_CENTROID", 3)));
    point.setField(pdal::Dimension::Id::LatitudeCentroid,
        convert<double>(s, "LATITUDE_CENTROID", 4));
    point.setField(pdal::Dimension::Id::ElevationCentroid,
        convert<double>(s, "ELEVATION_CENTROID", 5));
    point.setField(pdal::Dimension::Id::LongitudeLow,
        Utils::normalizeLongitude(convert<double>(s, "LONGITUDE_LOW", 6)));
    point.setField(pdal::Dimension::Id::LatitudeLow,
        convert<double>(s, "LATITUDE_LOW", 7));
    point.setField(pdal::Dimension::Id::ElevationLow,
        convert<double>(s, "ELEVATION_LOW", 8));
    point.setField(pdal::Dimension::Id::LongitudeHigh,
        Utils::normalizeLongitude(convert<double>(s, "LONGITUDE_HIGH", 9)));
    point.setField(pdal::Dimension::Id::LatitudeHigh,
        convert<double>(s, "LATITUDE_HIGH", 10));
    point.setField(pdal::Dimension::Id::ElevationHigh,
        convert<double>(s, "ELEVATION_HIGH", 11));

    double x, y, z;
    pdal::Dimension::Id xd, yd, zd;

    xd = m_layout->findDim("LONGITUDE_" + pointMap);
    yd = m_layout->findDim("LATITUDE_" + pointMap);
    zd = m_layout->findDim("ELEVATION_" + pointMap);

    x = point.getFieldAs<double>(xd);
    y = point.getFieldAs<double>(yd);
    z = point.getFieldAs<double>(zd);

    point.setField(pdal::Dimension::Id::X, x);
    point.setField(pdal::Dimension::Id::Y, y);
    point.setField(pdal::Dimension::Id::Z, z);
}


void Ilvis2Reader::ready(PointTableRef table)
{
    if (!m_metadataFile.empty())
    {
        try
        {
            m_mdReader.readMetadataFile(m_metadataFile, &m_metadata);
        }
        catch (const Ilvis2MetadataReader::error& err)
        {
            throwError(err.what());
        }
    }

    static const int HeaderSize = 2;
    std::string line;

    m_lineNum = 0;
    m_stream.reset(new std::ifstream(m_filename));
    m_layout = table.layout();
    m_resample = false;
    for (size_t i = 0; i < HeaderSize; ++i)
    {
        std::getline(*m_stream, line);
        m_lineNum++;
    }
}


bool Ilvis2Reader::processOne(PointRef& point)
{
    std::string line;

// Format:
// LVIS_LFID SHOTNUMBER TIME LONGITUDE_CENTROID LATITUDE_CENTROID ELEVATION_CENTROID LONGITUDE_LOW LATITUDE_LOW ELEVATION_LOW LONGITUDE_HIGH LATITUDE_HIGH ELEVATION_HIGH

    try
    {
        // This handles the second time through for this data line when we have
        // an "ALL" mapping and the high and low elevations are different.
        if (m_resample)
        {
            readPoint(point, m_fields, "HIGH");
            m_resample = false;
            return true;
        }

        if (!std::getline(*m_stream, line))
            return false;
        m_fields = Utils::split2(line, ' ');
        if (m_fields.size() != 12)
            throwError("Invalid format for line " +
                Utils::toString(m_lineNum) + ".  Expected 12 fields, got " +
                Utils::toString(m_fields.size()) + ".");

        double low_elev = convert<double>(m_fields, "ELEVATION_LOW", 8);
        double high_elev = convert<double>(m_fields, "ELEVATION_HIGH", 11);

        // write LOW point if specified, or for ALL
        if (m_mapping == IlvisMapping::LOW || m_mapping == IlvisMapping::ALL)
        {
            readPoint(point, m_fields, "LOW");
            // If we have ALL mapping and the high elevation is different
            // from that of the low elevation, we'll a second point with the
            // high elevation.
            if (m_mapping == IlvisMapping::ALL && (low_elev != high_elev))
                m_resample = true;
        }
        else if (m_mapping == IlvisMapping::HIGH)
            readPoint(point, m_fields, "HIGH");
    }
    catch (const error& err)
    {
        throwError(err.what());
    }
    return true;
}


point_count_t Ilvis2Reader::read(PointViewPtr view, point_count_t count)
{
    PointId idx = view->size();
    point_count_t numRead = 0;

    PointRef point = PointRef(*view, 0);
    while (numRead < count)
    {
        point.setPointId(idx++);
        if (!processOne(point))
            break;
        if (m_cb)
            m_cb(*view, idx);
        numRead++;
    }

    return numRead;
}


void Ilvis2Reader::done(PointTableRef table)
{
    m_stream.reset();
}

} // namespace pdal

