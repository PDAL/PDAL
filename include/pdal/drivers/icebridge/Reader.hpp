/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#ifndef INCLUDED_PDAL_DRIVER_ICEBRIDGE_READER_HPP
#define INCLUDED_PDAL_DRIVER_ICEBRIDGE_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/Options.hpp>
#include <pdal/Hdf5Handler.hpp>

#include <vector>

namespace pdal
{
namespace drivers
{
namespace icebridge
{

class icebridge_error : public pdal_error
{
public:
    icebridge_error(std::string const& msg)
        : pdal_error(msg)
    { }
};

class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.icebridge.reader", "Icebridge Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.icebridge.reader.html")   // TODO
    SET_STAGE_ENABLED(true)

    Reader(const Options& options);
    virtual ~Reader() { }

    virtual void initialize();
    static Options getDefaultOptions();
    static std::vector<Dimension> getDefaultDimensions();

    std::string getFileName() const;

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;

    pdal::StageRandomIterator*
        createRandomIterator(PointBuffer& buffer) const;

    std::map<std::string, Dimension> getDimensionNamesMap() const;

private:
    std::map<std::string, Dimension> m_dimensionNamesMap;
    Reader& operator=(const Reader&);   // Not implemented.
    Reader(const Reader&);              // Not implemented.
};

namespace iterators
{

class PDAL_DLL IteratorBase
{
public:
    IteratorBase(const pdal::drivers::icebridge::Reader& reader);
    virtual ~IteratorBase() { }

protected:
    boost::uint32_t readIcebridgeIntoBuffer(
            PointBuffer& pointBuffer,
            boost::uint64_t index);

    const boost::uint32_t m_numPoints;
    const std::map<std::string, Dimension> m_dimensionNamesMap;
    const Schema m_schema;

    Hdf5Handler m_hdf5Handler;

private:
    IteratorBase& operator=(const IteratorBase&);   // Not implemented.
    IteratorBase(const IteratorBase&);              // Not implemented.
};

namespace sequential
{

class PDAL_DLL Iterator : public ReaderSequentialIterator, public IteratorBase
{
public:
    Iterator(
            const pdal::drivers::icebridge::Reader& reader,
            PointBuffer& buffer);
    virtual ~Iterator() { }

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer& pointBuffer);
    bool atEndImpl() const;
};

} // sequential

namespace random
{

class PDAL_DLL Iterator : public ReaderRandomIterator, public IteratorBase
{
public:
    Iterator(
            const pdal::drivers::icebridge::Reader& reader,
            PointBuffer& buffer);
    virtual ~Iterator() { }

private:
    boost::uint64_t seekImpl(boost::uint64_t numSeek);
    boost::uint32_t readBufferImpl(PointBuffer& pointBuffer);
};

} // random

} // iterators

} // icebridge
} // drivers
} // pdal

#endif // INCLUDED_PDAL_DRIVER_ICEBRIDGE_READER_HPP
 
