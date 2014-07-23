/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#pragma once

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/Options.hpp>

#include <pdal/StageIterator.hpp>

#include <vector>

#include <boost/detail/endian.hpp>
#include <boost/scoped_ptr.hpp>


namespace pdal
{
namespace drivers
{
namespace terrasolid
{


enum TERRASOLID_Format_Type
{
    TERRASOLID_Format_1 = 20010712,
    TERRASOLID_Format_2 = 20020715,
    TERRASOLID_Format_Unknown = 999999999
};

struct TerraSolidHeader
{
    TerraSolidHeader() :
        HdrSize(0),
        HdrVersion(0),
        RecogVal(0),
        PntCnt(0),
        Units(0),
        OrgX(0),
        OrgY(0),
        OrgZ(0),
        Time(0),
        Color(0)
    {}

    boost::int32_t HdrSize;
    boost::int32_t HdrVersion;
    boost::int32_t RecogVal;
    char RecogStr[4];
    boost::int32_t PntCnt;
    boost::int32_t Units;
    double OrgX;
    double OrgY;
    double OrgZ;
    boost::int32_t Time;
    boost::int32_t Color;

};

typedef boost::scoped_ptr<TerraSolidHeader> TerraSolidHeaderPtr ;
class terrasolid_error : public pdal_error
{
public:

    terrasolid_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class PointDimensions
{
public:
    PointDimensions(const Schema& schema, std::string const& ns);

    DimensionPtr Time;
    DimensionPtr X;
    DimensionPtr Y;
    DimensionPtr Z;

    DimensionPtr Classification;
    DimensionPtr PointSourceId;
    DimensionPtr ReturnNumber;
    DimensionPtr Intensity;
    DimensionPtr Mark;
    DimensionPtr Flag;

    DimensionPtr Red;
    DimensionPtr Green;
    DimensionPtr Blue;
    DimensionPtr Alpha;
};
//
//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//   <string>filename  [required]
//

class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.terrasolid.reader", "TerraSolid Reader")
    SET_STAGE_ENABLED(true)

    Reader(const Options&);

    static Options getDefaultOptions();
    static std::vector<Dimension> getDefaultDimensions();
    std::string getFileName() const;
    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer& buffer) const;

    std::size_t getPointDataOffset() const
        { return m_offset; }
    boost::uint32_t getPointDataSize() const
        { return m_size; }
    point_count_t getNumPoints() const
        { return m_header->PntCnt; }

    // this is called by the stage's iterator
    uint32_t processBuffer(PointBuffer& PointBuffer, std::istream& stream,
        uint64_t numPointsLeft) const;

protected:
    inline TERRASOLID_Format_Type getFormat() const
    {
        return m_format;
    }

private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

    TerraSolidHeaderPtr m_header;
    TERRASOLID_Format_Type m_format;
    std::size_t m_offset;
    boost::uint32_t m_size;

    bool m_haveColor;
    bool m_haveTime;

    virtual void buildSchema(Schema *s);
};

namespace iterators
{

namespace sequential
{

class Reader : public pdal::ReaderSequentialIterator
{
public:
    Reader(const pdal::drivers::terrasolid::Reader& reader,
        PointBuffer& buffer);
    ~Reader();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::drivers::terrasolid::Reader& m_reader;
    std::istream* m_istream;
};


} // sequential

namespace random
{

class Reader : public pdal::ReaderRandomIterator
{
public:
    Reader(const pdal::drivers::terrasolid::Reader& reader,
        PointBuffer& buffer);
    ~Reader();

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);

    const pdal::drivers::terrasolid::Reader& m_reader;
    std::istream* m_istream;
};

} // random
} // iterators

} // namespace terrasolid
} // namespace drivers
} // namespace pdal

