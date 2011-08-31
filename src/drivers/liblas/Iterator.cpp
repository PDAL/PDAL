/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/drivers/liblas/Iterator.hpp>

#include <liblas/factory.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/drivers/las/Support.hpp>

namespace pdal { namespace drivers { namespace liblas {


//---------------------------------------------------------------------------
//
// LiblasIteratorBase
//
//---------------------------------------------------------------------------


LiblasIteratorBase::LiblasIteratorBase(const LiblasReader& reader)
    : m_reader(reader)
    , m_filename(reader.getFileName())
    , m_istream(NULL)
    , m_externalReader(NULL)
{
    m_istream = FileUtils::openFile(m_filename);

    ::liblas::ReaderFactory f;
    ::liblas::Reader extReader = f.CreateWithStream(*m_istream);
    m_externalReader = new ::liblas::Reader(extReader);

    return;
}


LiblasIteratorBase::~LiblasIteratorBase()
{
    delete m_externalReader;
    FileUtils::closeFile(m_istream);
}


::liblas::Reader& LiblasIteratorBase::getExternalReader() const
{
    return *m_externalReader;
}


const LiblasReader& LiblasIteratorBase::getReader() const
{
    return m_reader;
}


boost::uint32_t LiblasIteratorBase::myReadBuffer(PointBuffer& data)
{
    boost::uint32_t numPoints = data.getCapacity();
    boost::uint32_t i = 0;

    const Schema& schema = data.getSchema();

    ::pdal::drivers::las::PointFormat pointFormat = m_reader.getPointFormat();
    ::pdal::drivers::las::PointIndexes indexes(schema, pointFormat);

    for (i=0; i<numPoints; i++)
    {
        bool ok = getExternalReader().ReadNextPoint();
        if (!ok)
        {
            // we've hit the end of the file, or possibly something disasterous...
            break;
        }

        const ::liblas::Point& pt = getExternalReader().GetPoint();

        const boost::int32_t x = pt.GetRawX();
        const boost::int32_t y = pt.GetRawY();
        const boost::int32_t z = pt.GetRawZ();

        const boost::uint16_t intensity = pt.GetIntensity();
        const boost::int8_t returnNumber = (boost::int8_t)pt.GetReturnNumber();
        const boost::int8_t numberOfReturns = (boost::int8_t)pt.GetNumberOfReturns();
        const boost::int8_t scanDirFlag = (boost::int8_t)pt.GetScanDirection();
        const boost::int8_t edgeOfFlightLine = (boost::int8_t)pt.GetFlightLineEdge();
        const boost::uint8_t classification = pt.GetClassification().GetClass();
        const boost::int8_t scanAngleRank = pt.GetScanAngleRank();
        const boost::uint8_t userData = pt.GetUserData();
        const boost::uint16_t pointSourceId = pt.GetPointSourceID();
        
        data.setField(i, indexes.X, x);
        data.setField(i, indexes.Y, y);
        data.setField(i, indexes.Z, z);

        data.setField(i, indexes.Intensity, intensity);
        data.setField(i, indexes.ReturnNumber, returnNumber);
        data.setField(i, indexes.NumberOfReturns, numberOfReturns);
        data.setField(i, indexes.ScanDirectionFlag, scanDirFlag);
        data.setField(i, indexes.EdgeOfFlightLine, edgeOfFlightLine);
        data.setField(i, indexes.Classification, classification);
        data.setField(i, indexes.ScanAngleRank, scanAngleRank);
        data.setField(i, indexes.UserData, userData);
        data.setField(i, indexes.PointSourceId, pointSourceId);

        if (pdal::drivers::las::Support::hasTime(pointFormat))
        {
            const double time = pt.GetTime();
            
            data.setField(i, indexes.Time, time);
        }

        if (pdal::drivers::las::Support::hasColor(pointFormat))
        {
            const ::liblas::Color color = pt.GetColor();
            const boost::uint16_t red = color.GetRed();
            const boost::uint16_t green = color.GetGreen();
            const boost::uint16_t blue = color.GetBlue();

            data.setField(i, indexes.Red, red);
            data.setField(i, indexes.Green, green);
            data.setField(i, indexes.Blue, blue);
        }
        
        data.setNumPoints(i+1);
        if (pdal::drivers::las::Support::hasWave(pointFormat))
        {
            throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
        }
        
    }

    return i;
}


//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

SequentialIterator::SequentialIterator(const LiblasReader& reader)
    : LiblasIteratorBase(reader)
    , pdal::ReaderSequentialIterator(reader)
{
    return;
}


SequentialIterator::~SequentialIterator()
{
    return;
}


boost::uint64_t SequentialIterator::skipImpl(boost::uint64_t count)
{
    const boost::uint64_t newPos64 = getIndex() + count;

    // The liblas reader's seek() call only supports size_t, so we might
    // not be able to satisfy this request...

    if (newPos64 > std::numeric_limits<size_t>::max())
    {
        throw pdal_error("cannot support seek offsets greater than 32-bits");
    }

    // safe cast, since we just handled the overflow case
    size_t newPos = static_cast<size_t>(newPos64);
    
    getExternalReader().Seek(newPos);

    return count;
}



bool SequentialIterator::atEndImpl() const
{
    return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t SequentialIterator::readBufferImpl(PointBuffer& data)
{
    return myReadBuffer(data);
}


//---------------------------------------------------------------------------
//
// RandomIterator
//
//---------------------------------------------------------------------------

RandomIterator::RandomIterator(const LiblasReader& reader)
    : LiblasIteratorBase(reader)
    , pdal::ReaderRandomIterator(reader)
{
    return;
}


RandomIterator::~RandomIterator()
{
    return;
}


boost::uint64_t RandomIterator::seekImpl(boost::uint64_t newPos64)
{
    // The liblas reader's seek() call only supports size_t, so we might
    // not be able to satisfy this request...

    if (newPos64 > std::numeric_limits<size_t>::max())
    {
        throw pdal_error("cannot support seek offsets greater than 32-bits");
    }

    // safe cast, since we just handled the overflow case
    size_t newPos = static_cast<size_t>(newPos64);

    getExternalReader().Seek(newPos);

    return newPos;
}


boost::uint32_t RandomIterator::readBufferImpl(PointBuffer& data)
{
    return myReadBuffer(data);
}


} } } // namespaces
