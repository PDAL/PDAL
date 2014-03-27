/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

#include <pdal/drivers/caris/CloudIterator.hpp>

#include "Utils.hpp"

#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif

#include <pdal/PointBuffer.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Reader.hpp>

#include <boost/foreach.hpp>
#include <boost/uuid/uuid_io.hpp>

#ifdef _MSC_VER
#   pragma warning(pop)
#   pragma warning(disable : 4503)
#endif

namespace csar
{

using namespace csar::utils;

namespace
{
//************************************************************************
//! copy banded data to a pdal::PointBuffer
/*!
\param in_srcBuffer
    \li source buffer
\param in_srcOffset
    \li index of the first element to copy in \e in_srcBuffer
\param in_elementCount
    \li number of elements to copy
\param in_dimInfo
    \li description of source dimension
\param io_buffer
    \li buffer to copy to
\param in_pdalDim
    \li dimension to copy to
\return
    \li size (bytes) of an element of type \e in_type
*/
//************************************************************************
void copyToPointBuffer(
    void const* in_srcBuffer, size_t in_srcOffset,  size_t in_elementCount,
    CloudReader::DimInfo in_dimInfo,
    pdal::PointBuffer& io_buffer, pdal::Dimension const& in_pdalDim)
{
    assert(carisTypeToInterpretation(caris_type(in_dimInfo.dimension->type))
           == in_pdalDim.getInterpretation());
    assert(carisTypeToSize(caris_type(in_dimInfo.dimension->type))
           == in_pdalDim.getByteSize());

    const size_t elementSize = in_pdalDim.getByteSize();
    const size_t srcStride = in_dimInfo.dimension->tuple_length * elementSize;
    uint8_t const* src = static_cast<uint8_t const*>(in_srcBuffer)
                         + in_srcOffset * srcStride
                         + elementSize * in_dimInfo.tupleIndex;

    const size_t dstStride = io_buffer.getSchema().getByteSize();
    uint8_t * dst = io_buffer.getData(0) + in_pdalDim.getByteOffset();

    for (size_t i = 0;
            i < in_elementCount;
            ++i, src += srcStride, dst += dstStride)
    {
        ::memcpy(dst, src, elementSize);
    }
}
}

//************************************************************************
//! constructor
/*!
\param in_reader
    \li CloudReader to iterate
\param in_buffer
    \li ?
*/
//************************************************************************
CloudIterator::CloudIterator(pdal::PointBuffer & in_buffer, caris_cloud *cloud)
    : ReaderSequentialIterator(in_buffer)
    , m_itr(NULL)
    , m_dimInfo(in_reader.getDimInfo())
    , m_currentOffset(0)
{
    m_itr = caris_cloud_create_itr(cloud)
    throwIfItrError();
}

//************************************************************************
//! destructor
//************************************************************************
CloudIterator::~CloudIterator()
{
    if (m_itr)
        caris_itr_release(m_itr);
}

//! \copydoc pdal::ReaderSequentialIterator::readBufferImpl
boost::uint32_t CloudIterator::readBufferImpl(
    pdal::PointBuffer& io_buffer
)
{
    assert(m_itr);

    if (atEndImpl())
    {
        io_buffer.setNumPoints(0);
        return 0;
    }

    const int32_t numPoints = caris_itr_num_points(m_itr);
    assert(numPoints > m_currentOffset);
    const uint32_t remaining = numPoints - m_currentOffset;
    const uint32_t toProcess = std::min(remaining, io_buffer.getCapacity());

    io_buffer.setNumPoints(toProcess);

    pdal::Schema const& schema = io_buffer.getSchema();

    BOOST_FOREACH(pdal::Dimension const& pdalDim, schema.getDimensions().get<pdal::schema::index>())
    {
        boost::optional<CloudReader::DimInfo const&> dimInfo
            = getOptionalCRef(m_dimInfo, pdalDim.getUUID());

        if (!dimInfo)
        {
            // unknown dimension, possibly from a filter
            continue;
        }

        int32_t numDimElements = 0;
        void const* dimElements = caris_itr_read(m_itr, dimInfo->dimIndex, &numDimElements);

        throwIfItrError();

        assert(numDimElements == numPoints);
        copyToPointBuffer(
            dimElements, m_currentOffset, toProcess, *dimInfo,
            io_buffer, pdalDim);
    }

    m_currentOffset += toProcess;
    if (m_currentOffset == numPoints)
    {
        m_currentOffset = 0;
        caris_itr_next(m_itr);
    }

    throwIfItrError();

    return toProcess;
}

//! \copydoc pdal::ReaderSequentialIterator::skipImpl
boost::uint64_t CloudIterator::skipImpl(boost::uint64_t in_pointNum)
{
    uint64_t nextOffset = m_currentOffset + in_pointNum;

    while (!caris_itr_done(m_itr))
    {
        const int32_t numPoints = caris_itr_num_points(m_itr);

        if (nextOffset < numPoints)
        {
            m_currentOffset = static_cast<int32_t>(nextOffset);
            break;
        }

        nextOffset -= numPoints;
        caris_itr_next(m_itr);
    }

    throwIfItrError();

    return in_pointNum - (nextOffset - m_currentOffset);
}

//! \copydoc pdal::ReaderSequentialIterator::atEndImpl
bool CloudIterator::atEndImpl() const
{
    return caris_itr_done(m_itr) != 0;
}

//************************************************************************
//! throw a pdal::pdal_error if an error has occured in the wrapped caris_itr
//************************************************************************
void CloudIterator::throwIfItrError() const
{
    if (int status = caris_itr_status(m_itr))
    {
        std::string msg = caris_status_message(status, getStage().isDebug());
        throw pdal::pdal_error(msg);
    }
}

} // namespace
