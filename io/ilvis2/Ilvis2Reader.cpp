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
#include <algorithm>

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
    options.add("mapping", "ALL", "The point to extract from the shot: LOW, HIGH, or ALL.  ALL creates 1 or 2 points per shot, depending on if LOW and HIGH are the same.");
    options.add("filename", "", "The file to read from");
    options.add("metadata", "", "The metadata file to read from");

    return options;
}

void Ilvis2Reader::processOptions(const Options& options)
{
    m_mapping = options.getValueOrDefault<std::string>("mapping", "ALL");
}

void Ilvis2Reader::addDimensions(PointLayoutPtr layout)
{
    using namespace pdal::Dimension::Type;
    layout->registerOrAssignDim("LVIS_LFID", Unsigned64);
    layout->registerOrAssignDim("SHOTNUMBER", Unsigned64);
    layout->registerDim(pdal::Dimension::Id::GpsTime);
    layout->registerOrAssignDim("LONGITUDE_CENTROID", Double);
    layout->registerOrAssignDim("LATITUDE_CENTROID", Double);
    layout->registerOrAssignDim("ELEVATION_CENTROID", Double);
    layout->registerOrAssignDim("LONGITUDE_LOW", Double);
    layout->registerOrAssignDim("LATITUDE_LOW", Double);
    layout->registerOrAssignDim("ELEVATION_LOW", Double);
    layout->registerOrAssignDim("LONGITUDE_HIGH", Double);
    layout->registerOrAssignDim("LATITUDE_HIGH", Double);
    layout->registerOrAssignDim("ELEVATION_HIGH", Double);
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    layout->registerDim(pdal::Dimension::Id::Z);
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
    SpatialReference ref("EPSG:4385");
    setSpatialReference(ref);
}


void Ilvis2Reader::ready(PointTableRef)
{
    m_stream.reset(new ILeStream(m_filename));
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
  if (longitude > 180.0)
  {
    return longitude - 360.0;
  }
  else
  {
    return longitude;
  }
}


void Ilvis2Reader::readPoint(PointViewPtr view, PointId nextId, StringList s, std::string pointMap)
{
  PointLayoutPtr layout = view->layout();

  std::string name("LVIS_LFID");
  view->setField(layout->findProprietaryDim(name), nextId, convert<unsigned int>(s, name, 0));

  name = "SHOTNUMBER";
  view->setField(layout->findProprietaryDim(name), nextId, convert<unsigned int>(s, name, 1));

  view->setField(pdal::Dimension::Id::GpsTime, nextId, convert<double>(s, name, 2));

  name = "LONGITUDE_CENTROID";
  view->setField(layout->findProprietaryDim(name), nextId, convertLongitude(convert<double>(s, name, 3)));

  name = "LATITUDE_CENTROID";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 4));

  name = "ELEVATION_CENTROID";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 5));

  name = "LONGITUDE_LOW";
  view->setField(layout->findProprietaryDim(name), nextId, convertLongitude(convert<double>(s, name, 6)));

  name = "LATITUDE_LOW";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 7));

  name = "ELEVATION_LOW";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 8));

  name = "LONGITUDE_HIGH";
  view->setField(layout->findProprietaryDim(name), nextId, convertLongitude(convert<double>(s, name, 9)));

  name = "LATITUDE_HIGH";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 10));

  name = "ELEVATION_HIGH";
  view->setField(layout->findProprietaryDim(name), nextId, convert<double>(s, name, 11));

  double x, y, z;
  pdal::Dimension::Id::Enum xd, yd, zd;

  xd = layout->findProprietaryDim("LONGITUDE_" + pointMap);
  yd = layout->findProprietaryDim("LATITUDE_" + pointMap);
  zd = layout->findProprietaryDim("ELEVATION_" + pointMap);

  x = view->getFieldAs<double>(xd, nextId);
  y = view->getFieldAs<double>(yd, nextId);
  z = view->getFieldAs<double>(zd, nextId);

  view->setField(pdal::Dimension::Id::X, nextId, x);
  view->setField(pdal::Dimension::Id::Y, nextId, y);
  view->setField(pdal::Dimension::Id::Z, nextId, z);
}

point_count_t Ilvis2Reader::read(PointViewPtr view, point_count_t count)
{
    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size();
    PointId idx = m_index;
    double low_elev;
    double high_elev;
    point_count_t numRead = 0;

    // do any skipping to get to our m_index position;
    m_stream.reset(new ILeStream(m_filename));

    size_t HEADERSIZE(2);
    size_t skip_lines(std::max(HEADERSIZE, (size_t)m_index));
    size_t line_no(1);
    for (std::string line; std::getline(*m_stream->stream(), line); line_no++)
    {
        if (line_no <= skip_lines)
        {
            continue;
        }

        StringList s = Utils::split2(line, ' ');
//        # LVIS_LFID SHOTNUMBER TIME LONGITUDE_CENTROID LATITUDE_CENTROID ELEVATION_CENTROID LONGITUDE_LOW LATITUDE_LOW ELEVATION_LOW LONGITUDE_HIGH LATITUDE_HIGH ELEVATION_HIGH
        unsigned long u64(0);
        if (s.size() != 12)
        {
            std::stringstream oss;
            oss << "Unable to split proper number of fields. Expected 12, got " << s.size();
            throw pdal_error(oss.str());
        }

        std::string name("ELEVATION_LOW");
        low_elev = convert<double>(s, name, 8);
        name = "ELEVATION_HIGH";
        high_elev = convert<double>(s, name, 11);

        // write LOW point if specified, or for ALL
        if (m_mapping == "LOW" || m_mapping == "ALL")
        {
          readPoint(view, nextId, s, "LOW");
          nextId++;
        }

        // write HIGH point if specified, or for ALL, but only if it's not the same as the low point
        if (m_mapping == "HIGH" || (m_mapping == "ALL" && low_elev != high_elev))
        {
          readPoint(view, nextId, s, "HIGH");
          nextId++;
        }

        if (m_cb)
            m_cb(*view, nextId);
    }
    m_index = nextId;
    numRead = nextId;
    return numRead;
}

} // namespace pdal

