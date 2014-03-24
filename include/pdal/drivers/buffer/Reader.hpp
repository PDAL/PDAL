/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#ifndef INCLUDED_DRIVERS_BUFFER_READER_HPP
#define INCLUDED_DRIVERS_BUFFER_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/PointBuffer.hpp>


namespace pdal
{
namespace drivers
{
namespace buffer
{


// The BufferReader doesn't read from disk, but instead just grabs data from an
// existing PointBuffer.
//
// This reader knows about 3 fields (Dimensions):
//    X,Y,Z - floats
//
class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.buffer.reader", "Buffer Reader")

    Reader(const Options& options, const PointBuffer& buffer);

    virtual void initialize();
    static Options getDefaultOptions();

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer& buffer) const;

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& data, boost::uint64_t index) const;

private:
    PointBuffer m_buffer;
    Bounds<double> m_bounds;
    boost::uint64_t m_numPoints;
    Schema m_schema;

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

};


namespace iterators
{
namespace sequential
{

class PDAL_DLL Reader : public pdal::ReaderSequentialIterator
{
public:
    Reader(pdal::drivers::buffer::Reader const& reader, PointBuffer& buffer);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    pdal::drivers::buffer::Reader const& m_reader;
};

}
} // iterators::sequential

namespace iterators
{
namespace random
{

class PDAL_DLL Reader : public pdal::ReaderRandomIterator
{
public:
    Reader(pdal::drivers::buffer::Reader const& reader, PointBuffer& buffer);

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);

    pdal::drivers::buffer::Reader const& m_reader;
};

}
} // iterators::random

}
}
} // namespaces


#endif
