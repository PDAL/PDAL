/******************************************************************************
* Copyright (c) 2011, Michael S. Rosen (michael.rosen@gmail.com)
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

#include <pdal/Bounds.hpp>

#include <lidar/PointSource.h>

namespace pdal
{
class PointBuffer;
}
namespace LizardTech
{
class PointSource;
}

namespace pdal
{
namespace drivers
{
namespace mrsid
{


// The MrSIDReader wraps LT's PointSource abstraction
//
class PDAL_DLL MrsidReader : public pdal::Reader
{

public:
    SET_STAGE_NAME("drivers.mrsid.reader", "MrSID Reader")
    SET_STAGE_LINK("http://www.pdal.io/stages/drivers.mrsid.reader.html")
    SET_STAGE_ENABLED(true)

    virtual ~MrsidReader();
    MrsidReader() : Writer() {};
    MrsidReader(LizardTech::PointSource *ps);

    static Options getDefaultOptions();
    static std::vector<Dimension> getDefaultDimensions();
    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& data, boost::uint64_t index) const;

private:
    LizardTech::PointSource *m_PS;
    LizardTech::PointIterator *m_iter;

    virtual void initialize();
    int SchemaToPointInfo(const Schema &schema, LizardTech::PointInfo &pointInfo) const;
    Dimension LTChannelToPDalDimension(const LizardTech::ChannelInfo & channel, pdal::Schema const& dimensions) const;
    MrsidReader& operator=(const MrsidReader&); // not implemented
    MrsidReader(const MrsidReader&); // not implemented
};

namespace iterators
{

namespace sequential
{

class MrsidReader : public pdal::ReaderSequentialIterator
{
public:
    MrsidReader(const pdal::drivers::mrsid::MrsidReader& reader, PointBuffer& buffer, boost::uint32_t numPoints);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    point_count_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
    boost::uint32_t m_numPoints;

    const pdal::drivers::mrsid::MrsidReader& m_reader;
};

} // sequential

namespace random
{

class MrsidReader : public pdal::ReaderRandomIterator
{
public:
    MrsidReader(const pdal::drivers::mrsid::MrsidReader& reader,
           PointBuffer& buffer, boost::uint32_t numPoints);

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);

    const pdal::drivers::mrsid::MrsidReader& m_reader;
    boost::uint32_t m_numPoints;

};


} // random
} // iterators


}
}
} // namespaces
