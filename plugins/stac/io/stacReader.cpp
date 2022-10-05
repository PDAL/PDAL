/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include "stacReader.hpp"

#include <pdal/util/ProgramArgs.hpp>


namespace pdal
{

  static PluginInfo const stacinfo
  {
    "readers.stac",
    "STAC Reader",
    "http://pdal.io/stages/readers.stac.html"
  };


  CREATE_SHARED_STAGE(StacReader, stacinfo)

  std::string StacReader::getName() const { return stacinfo.name; }

  void StacReader::addArgs(ProgramArgs& args)
  {

    args.add("z_scale", "Z Scaling", m_scale_z, 1.0);
    args.add("z_scale", "Z Scaling", m_scale_z, 1.0);

  }

  void StacReader::addDimensions(PointLayoutPtr layout)
  {

    layout->registerDim(Dimension::Id::X);

    layout->registerDim(Dimension::Id::Y);

    layout->registerDim(Dimension::Id::Z);

    layout->registerOrAssignDim("MyData", Dimension::Type::Unsigned64);

  }

  void StacReader::ready(PointTableRef)

  {

    m_index = 0;

    SpatialReference ref("EPSG:4385");

    setSpatialReference(ref);

  }

  template <typename T>

  T convert(const StringList& s, const std::string& name, size_t fieldno)
  {

      T output;

      bool bConverted = Utils::fromString(s[fieldno], output);

      if (!bConverted)

      {

          std::stringstream oss;

          oss << "Unable to convert " << name << ", " << s[fieldno] <<

              ", to double";

          throw pdal_error(oss.str());

      }
      return output;
  }

  point_count_t StacReader::read(PointViewPtr view, point_count_t count)
  {

    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;

    m_stream.reset(new ILeStream(m_filename));

    size_t HEADERSIZE(1);
    size_t skip_lines((std::max)(HEADERSIZE, (size_t)m_index));
    size_t line_no(1);

    for (std::string line; std::getline(*m_stream->stream(), line); line_no++)
    {
      if (line_no <= skip_lines)
      {

        continue;

      }
      // StacReader format:  X::Y::Z::Data
      StringList s = Utils::split2(line, ':');


      unsigned long u64(0);

      if (s.size() != 4)

      {

        std::stringstream oss;

        oss << "Unable to split proper number of fields.  Expected 4, got "

            << s.size();

        throw pdal_error(oss.str());

      }


      std::string name("X");

      view->setField(Dimension::Id::X, nextId, convert<double>(s, name, 0));


      name = "Y";

      view->setField(Dimension::Id::Y, nextId, convert<double>(s, name, 1));


      name = "Z";

      double z = convert<double>(s, name, 2) * m_scale_z;

      view->setField(Dimension::Id::Z, nextId, z);


      name = "MyData";

      view->setField(layout->findProprietaryDim(name),

                     nextId,

                     convert<unsigned int>(s, name, 3));


      nextId++;

      if (m_cb)

        m_cb(*view, nextId);

    }

    m_index = nextId;

    numRead = nextId;


    return numRead;

  }


  void StacReader::done(PointTableRef)

  {

    m_stream.reset();

  }


} //namespace pdal