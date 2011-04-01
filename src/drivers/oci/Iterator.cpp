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

#include <libpc/drivers/oci/Iterator.hpp>

#include <liblas/factory.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/Utils.hpp>
#include <libpc/drivers/oci/Reader.hpp>

namespace libpc { namespace drivers { namespace oci {

IteratorBase::IteratorBase(const Reader& reader)
    : m_at_end(false)
    , m_reader(reader)
{
    oci::Options& options = m_reader.getOptions();
    
    return;
}


IteratorBase::~IteratorBase()
{
}



const Reader& IteratorBase::getReader() const
{
    return m_reader;
}


boost::uint32_t IteratorBase::readBuffer(PointBuffer& data)
{
    boost::uint32_t numPoints = data.getCapacity();
    
    bool read = m_reader.fetchNext();
    Block* block = m_reader.getBlock();
    
    if (!read)
    {
        m_at_end = true;
        return 0;
    }
        std::cout << "fetched" << std::endl;
    // boost::uint32_t i = 0;
    // 
    // const Schema& schema = data.getSchema();
    // 
    // const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    // const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    // const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    // 
    // const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);
    // const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
    // const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns, Dimension::Uint8);
    // const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag, Dimension::Uint8);
    // const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8);
    // const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
    // const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank, Dimension::Int8);
    // const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData, Dimension::Uint8);
    // const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
    // 
    // const int indexTime = (getReader().hasTimeData() ? schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double) : 0);
    // 
    // const int indexRed = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16) : 0);
    // const int indexGreen = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16) : 0);
    // const int indexBlue = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16) : 0);
    // 
    // //const int indexWavePacketDescriptorIndex = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    // //const int indexWaveformDataOffset = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    // //const int indexReturnPointWaveformLocation = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    // //const int indexWaveformXt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    // //const int indexWaveformYt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    // //const t indexWaveformZt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);
    // 
    // for (i=0; i<numPoints; i++)
    // {
    //     bool ok = getExternalReader().ReadNextPoint();
    //     if (!ok)
    //     {
    //         throw libpc_error("liblas reader failed to retrieve point");
    //     }
    // 
    //     const ::liblas::Point& pt = getExternalReader().GetPoint();
    // 
    //     const boost::int32_t x = pt.GetRawX();
    //     const boost::int32_t y = pt.GetRawY();
    //     const boost::int32_t z = pt.GetRawZ();
    // 
    //     const boost::uint16_t intensity = pt.GetIntensity();
    //     const boost::int8_t returnNumber = (boost::int8_t)pt.GetReturnNumber();
    //     const boost::int8_t numberOfReturns = (boost::int8_t)pt.GetNumberOfReturns();
    //     const boost::int8_t scanDirFlag = (boost::int8_t)pt.GetScanDirection();
    //     const boost::int8_t edgeOfFlightLine = (boost::int8_t)pt.GetFlightLineEdge();
    //     const boost::uint8_t classification = pt.GetClassification().GetClass();
    //     const boost::int8_t scanAngleRank = pt.GetScanAngleRank();
    //     const boost::uint8_t userData = pt.GetUserData();
    //     const boost::uint16_t pointSourceId = pt.GetPointSourceID();
    //     
    //     data.setField(i, indexX, x);
    //     data.setField(i, indexY, y);
    //     data.setField(i, indexZ, z);
    // 
    //     data.setField(i, indexIntensity, intensity);
    //     data.setField(i, indexReturnNumber, returnNumber);
    //     data.setField(i, indexNumberOfReturns, numberOfReturns);
    //     data.setField(i, indexScanDirectionFlag, scanDirFlag);
    //     data.setField(i, indexEdgeOfFlightLine, edgeOfFlightLine);
    //     data.setField(i, indexClassification, classification);
    //     data.setField(i, indexScanAngleRank, scanAngleRank);
    //     data.setField(i, indexUserData, userData);
    //     data.setField(i, indexPointSourceId, pointSourceId);
    // 
    //     if (getReader().hasTimeData())
    //     {
    //         const double time = pt.GetTime();
    //         
    //         data.setField(i, indexTime, time);
    //     }
    // 
    //     if (getReader().hasColorData())
    //     {
    //         const ::liblas::Color color = pt.GetColor();
    //         const boost::uint16_t red = color.GetRed();
    //         const boost::uint16_t green = color.GetGreen();
    //         const boost::uint16_t blue = color.GetBlue();
    // 
    //         data.setField(i, indexRed, red);
    //         data.setField(i, indexGreen, green);
    //         data.setField(i, indexBlue, blue);
    //     }
    //     
    //     data.setNumPoints(i+1);
    //     if (getReader().hasWaveData())
    //     {
    //         throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    //     }
    //     
    // }

    return numPoints;
}


//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

SequentialIterator::SequentialIterator(const Reader& reader)
    : IteratorBase(reader)
    , libpc::SequentialIterator(reader)
{
    return;
}


SequentialIterator::~SequentialIterator()
{
    return;
}


boost::uint64_t SequentialIterator::skipImpl(boost::uint64_t count)
{
    // const boost::uint64_t newPos64 = getIndex() + count;
    // 
    // // The liblas reader's seek() call only supports size_t, so we might
    // // not be able to satisfy this request...
    // 
    // if (newPos64 > std::numeric_limits<size_t>::max())
    // {
    //     throw libpc_error("cannot support seek offsets greater than 32-bits");
    // }
    // 
    // // safe cast, since we just handled the overflow case
    // size_t newPos = static_cast<size_t>(newPos64);
    // 
    // getExternalReader().Seek(newPos);

    return 0;
}



bool SequentialIterator::atEndImpl() const
{
    return m_at_end; 
    // return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t SequentialIterator::readImpl(PointBuffer& data)
{
    return readBuffer(data);
}


//---------------------------------------------------------------------------
//
// RandomIterator
//
//---------------------------------------------------------------------------


} } } // namespaces
