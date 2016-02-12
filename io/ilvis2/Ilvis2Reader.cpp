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
#include <pdal/util/FileUtils.hpp>

#include <algorithm>
#include <cmath>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.ilvis2",
    "ILVIS2 Reader",
    "http://pdal.io/stages/readers.ilvis2.html" );

CREATE_STATIC_PLUGIN(1, 0, Ilvis2Reader, Reader, s_info)

std::string Ilvis2Reader::getName() const { return s_info.name; }

Options Ilvis2Reader::getDefaultOptions()
{
    Options options;
    options.add("mapping", "all", "The point to extract from the shot: "
        "low, high, or all.  'all' creates 1 or 2 points per shot, "
        "depending on if LOW and HIGH are the same.");
    options.add("filename", "", "The file to read from");
    options.add("metadata", "", "The metadata file to read from");

    return options;
}

void Ilvis2Reader::processOptions(const Options& options)
{
    std::string mapping =
        options.getValueOrDefault<std::string>("mapping", "all");
    mapping = Utils::toupper(mapping);
    m_mapping = parser.parseMapping(mapping);
    if (m_mapping == INVALID)
    {
        std::ostringstream oss;
        oss << "Invalid value for option for mapping: '" <<
            mapping << "'.  Value values are 'low', 'high' and 'all'.";
        throw pdal_error(oss.str());
    }

    m_metadataFile =
        options.getValueOrDefault<std::string>("metadata", "");
    if (!m_metadataFile.empty() && ! FileUtils::fileExists(m_metadataFile))
    {
        std::ostringstream oss;
        oss << "Invalid metadata file: '" << m_metadataFile << "'";
        throw pdal_error(oss.str());
    }
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


Dimension::IdList Ilvis2Reader::getDefaultDimensions()
{
    using namespace pdal::Dimension;
    Dimension::IdList ids;

    ids.push_back(Id::GpsTime);
    ids.push_back(Id::Y);
    ids.push_back(Id::X);
    ids.push_back(Id::Z);
    return ids;
}


void Ilvis2Reader::initialize(PointTableRef)
{
    // Data are WGS84 (4326) with ITRF2000 datum (6656)
    // See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html for
    // background
    SpatialReference ref("EPSG:4326");
    setSpatialReference(m_metadata, ref);
}


template <typename T>
T convert(const StringList& s, const std::string& name, size_t fieldno)
{
    T output;
    if (!Utils::fromString(s[fieldno], output))
    {
        std::stringstream oss;
        oss << "Unable to convert " << name << ", " << s[fieldno] <<
            ", to double";
        throw pdal_error(oss.str());
    }

    return output;
}


// If longitude between 0-180, just return it, degrees east; if between 180
// and 360, subtract 360 to get negative value.
double Ilvis2Reader::convertLongitude(double longitude)
{
    longitude = fmod(longitude, 360.0);
    if (longitude <= -180)
        longitude += 360;
    else if (longitude > 180)
        longitude -= 360;
    return longitude;
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
        convertLongitude(convert<double>(s, "LONGITUDE_CENTROID", 3)));
    point.setField(pdal::Dimension::Id::LatitudeCentroid,
        convert<double>(s, "LATITUDE_CENTROID", 4));
    point.setField(pdal::Dimension::Id::ElevationCentroid,
        convert<double>(s, "ELEVATION_CENTROID", 5));
    point.setField(pdal::Dimension::Id::LongitudeLow,
        convertLongitude(convert<double>(s, "LONGITUDE_LOW", 6)));
    point.setField(pdal::Dimension::Id::LatitudeLow,
        convert<double>(s, "LATITUDE_LOW", 7));
    point.setField(pdal::Dimension::Id::ElevationLow,
        convert<double>(s, "ELEVATION_LOW", 8));
    point.setField(pdal::Dimension::Id::LongitudeHigh,
        convertLongitude(convert<double>(s, "LONGITUDE_HIGH", 9)));
    point.setField(pdal::Dimension::Id::LatitudeHigh,
        convert<double>(s, "LATITUDE_HIGH", 10));
    point.setField(pdal::Dimension::Id::ElevationHigh,
        convert<double>(s, "ELEVATION_HIGH", 11));

    double x, y, z;
    pdal::Dimension::Id::Enum xd, yd, zd;

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
    static const int HeaderSize = 2;
    std::string line;

    m_lineNum = 0;
    m_stream.open(m_filename);
    m_layout = table.layout();
    m_resample = false;
    for (size_t i = 0; i < HeaderSize; ++i)
    {
        std::getline(m_stream, line);
        m_lineNum++;
    }
}


bool Ilvis2Reader::processOne(PointRef& point)
{
    std::string line;

// Format:
// LVIS_LFID SHOTNUMBER TIME LONGITUDE_CENTROID LATITUDE_CENTROID ELEVATION_CENTROID LONGITUDE_LOW LATITUDE_LOW ELEVATION_LOW LONGITUDE_HIGH LATITUDE_HIGH ELEVATION_HIGH

    // This handles the second time through for this data line when we have
    // an "ALL" mapping and the high and low elevations are different.
    if (m_resample)
    {
        readPoint(point, m_fields, "HIGH");
        m_resample = false;
        return true;
    }

    if (!std::getline(m_stream, line))
        return false;
    m_fields = Utils::split2(line, ' ');
    if (m_fields.size() != 12)
    {
        std::stringstream oss;
        oss << getName() << ": Invalid format for line " << m_lineNum <<
            ".  Expected 12 fields, got " << m_fields.size() << ".";
        throw pdal_error(oss.str());
    }

    double low_elev = convert<double>(m_fields, "ELEVATION_LOW", 8);
    double high_elev = convert<double>(m_fields, "ELEVATION_HIGH", 11);

    // write LOW point if specified, or for ALL
    if (m_mapping == LOW || m_mapping == ALL)
    {
        readPoint(point, m_fields, "LOW");
        // If we have ALL mapping and the high elevation is different
        // from that of the low elevation, we'll a second point with the
        // high elevation.
        if (m_mapping == ALL && (low_elev != high_elev))
            m_resample = true;
    }
    else if (m_mapping == HIGH)
        readPoint(point, m_fields, "HIGH");
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
    if (!m_metadataFile.empty())
    {
        m_mdReader.readMetadataFile(m_metadataFile, &m_metadata);
    }

}

} // namespace pdal

