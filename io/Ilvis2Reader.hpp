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

#include <pdal/pdal_features.hpp> // For PDAL_HAVE_LIBXML2
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/IStream.hpp>
#include <map>

#ifndef PDAL_HAVE_LIBXML2
namespace pdal
{
  class Ilvis2MetadataReader
  {
  public:
      inline void readMetadataFile(std::string filename, pdal::MetadataNode* m) {};
  };
}
#else
    #include "Ilvis2MetadataReader.hpp"
#endif

namespace pdal
{
class PDAL_EXPORT Ilvis2Reader : public Reader, public Streamable
{
public:
    enum class IlvisMapping
    {
      INVALID,
      LOW,
      HIGH,
      ALL
    };

    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    Ilvis2Reader();
    ~Ilvis2Reader();

    std::string getName() const;

private:
    std::unique_ptr<std::ifstream> m_stream;
    IlvisMapping m_mapping;
    StringList m_fields;
    size_t m_lineNum;
    bool m_resample;
    PointLayoutPtr m_layout;
    std::string m_metadataFile;
    Ilvis2MetadataReader m_mdReader;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);

    virtual void readPoint(PointRef& point, StringList s, std::string pointMap);
};

std::ostream& operator<<(std::ostream& out,
    const Ilvis2Reader::IlvisMapping& mval);

} // namespace pdal
