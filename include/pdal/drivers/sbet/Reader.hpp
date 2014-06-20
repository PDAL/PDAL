/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#ifndef INCLUDED_DRIVERS_SBET_READER_HPP
#define INCLUDED_DRIVERS_SBET_READER_HPP

#include <pdal/PointBuffer.hpp>
#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>


namespace pdal
{
namespace drivers
{
namespace sbet
{


class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.sbet.reader", "SBET Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.sbet.reader.html")
    SET_STAGE_ENABLED(true)

    Reader(const Options&);
    ~Reader();

    virtual void initialize();
    static Options getDefaultOptions();
    static std::vector<Dimension> getDefaultDimensions();

    std::string getFileName() const;

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer&) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const;

}; // class Reader


namespace iterators
{


class PDAL_DLL IteratorBase
{
public:
    IteratorBase(const pdal::drivers::sbet::Reader&, PointBuffer&);
    ~IteratorBase();

protected:
    boost::uint32_t readSbetIntoBuffer(PointBuffer&, const boost::uint64_t);
    std::istream* m_istream;
    const boost::uint64_t m_numPoints;
    const Schema m_schema;
    PointBuffer m_readBuffer;

private:
    IteratorBase& operator=(const IteratorBase&); // not implemented
    IteratorBase(const IteratorBase&); // not implemented
}; // class Iterator Base


namespace sequential
{


class PDAL_DLL Iterator : public pdal::ReaderSequentialIterator, public IteratorBase
{
public:
    Iterator(const pdal::drivers::sbet::Reader&, PointBuffer&);
    ~Iterator();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
};


} // namespace sequential


namespace random
{


class PDAL_DLL Iterator : public pdal::ReaderRandomIterator, public IteratorBase
{
public:
    Iterator(const pdal::drivers::sbet::Reader&, PointBuffer&);
    ~Iterator();

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
};


} // namespace random
} // namespace iterators

}
}
} // namespace pdal::drivers::sbet


#endif // INCLUDED_DRIVERS_SBET_READER_HPP
