/******************************************************************************
* Copyright (c) 2014, Hobu Inc., hobu.inc@gmail.com
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

#include <pdal/Dimension.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Utils.hpp>
#include <pdal/drivers/oci/OciSeqIterator.hpp>

#ifdef USE_PDAL_PLUGIN_OCI
MAKE_READER_CREATOR(ociReader, pdal::drivers::oci::Reader)
CREATE_READER_PLUGIN(oci, pdal::drivers::oci::Reader)
#endif


namespace pdal
{
namespace drivers
{
namespace oci
{


namespace iterators
{
namespace sequential
{


void OciSeqIterator::readBlob(Statement stmt, BlockPtr block)
{
    boost::uint32_t amountRead = 0;
    boost::uint32_t blobLength = stmt->GetBlobLength(block->locator);

    if (block->chunk.size() < blobLength)
        block->chunk.resize(blobLength);

    if (!stmt->ReadBlob(block->locator, (void*)(block->chunk.data()),
        block->chunk.size() , &amountRead))
        throw pdal_error("Did not read all blob data!");
}


point_count_t OciSeqIterator::read(PointBuffer& buffer, BlockPtr block,
    point_count_t numPts)
{
    if (block->orientation() == Orientation::DimensionMajor)
        return readDimMajor(buffer, block, numPts);
    else if (block->orientation() == Orientation::PointMajor)
        return readPointMajor(buffer, block, numPts);
    throw pdal_error("Unsupported orientation.");
}


point_count_t OciSeqIterator::readDimMajor(PointBuffer& buffer, BlockPtr block,
    point_count_t numPts)
{
    point_count_t numRemaining = block->numRemaining();
    PointId startId = buffer.size();
    point_count_t blockRemaining;
    point_count_t numRead = 0;

    for (size_t d = 0; d < m_dims.size(); ++d)
    {
        PointId nextId = startId;
        char *pos = seekDimMajor(d, block);
        blockRemaining = numRemaining;
        numRead = 0;
        while (numRead < numPts && blockRemaining > 0)
        {
            buffer.setRawField(m_dims[d], nextId, pos);
            pos += m_dims[d]->getByteSize();
            nextId++;
            numRead++;
            blockRemaining--;
        }
    }
    block->setNumRemaining(blockRemaining);
    return numRead;
}


point_count_t OciSeqIterator::readPointMajor(PointBuffer& buffer,
    BlockPtr block, point_count_t numPts)
{
    size_t numRemaining = block->numRemaining();
    PointId nextId = buffer.size();
    point_count_t numRead = 0;

    char *pos = seekPointMajor(block);
    while (numRead < numPts && numRemaining > 0)
    {
        for (size_t d = 0; d < m_dims.size(); ++d)
        {
            buffer.setRawField(m_dims[d], nextId, pos);
            pos += m_dims[d]->getByteSize();
        }
        numRemaining--;
        nextId++;
        numRead++;
    }
    block->setNumRemaining(numRemaining);
    return numRead;
}


char *OciSeqIterator::seekDimMajor(size_t dimIdx, BlockPtr block)
{
    size_t size = 0;
    for (size_t d = 0; d < dimIdx; ++d)
        size += m_dims[d]->getByteSize();    
    return block->data() +
        (size * block->numPoints()) +
        (m_dims[dimIdx]->getByteSize() * block->numRead());
}


char *OciSeqIterator::seekPointMajor(BlockPtr block)
{
    size_t size = 0;
    for (size_t d = 0; d < m_dims.size(); ++d)
        size += m_dims[d]->getByteSize();    
    return block->data() + (block->numRead() * size);
}


point_count_t OciSeqIterator::readImpl(PointBuffer& buffer, point_count_t count)
{
    if (atEndImpl())
        return 0;

    point_count_t totalNumRead = 0;
    while (totalNumRead < count)
    {
        if (m_block->numRemaining() == 0)
            if (!readOci(m_stmt, m_block))
                return totalNumRead;
        PointId bufBegin = buffer.size();
        point_count_t numRead = read(buffer, m_block, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
        if (m_normalizeXYZ)
            normalize(buffer, m_block, bufBegin, bufEnd);
    }
    return totalNumRead;
}


uint64_t OciSeqIterator::skipImpl(uint64_t count)
{
    BlockPtr block = m_block;

    boost::uint64_t initialCount = count;
    while (count)
    {
        point_count_t numRem = block->numRemaining();
        point_count_t blockCount = std::min((point_count_t)count, numRem);
        block->setNumRemaining(numRem - blockCount);
        count -= blockCount;
        if (count > 0)
            if (!readOci(m_stmt, m_block))
                break;
    }
    return initialCount - count;
}


void OciSeqIterator::normalize(PointBuffer& buffer, BlockPtr block,
    PointId begin, PointId end)
{
    // If the scales/offsets of this block are the same as the schema's,
    // then we're done.
    if (Utils::compare_distance(m_dimX->getNumericScale(), block->xScale()) &&
        Utils::compare_distance(m_dimX->getNumericOffset(), block->xOffset()) &&
        Utils::compare_distance(m_dimY->getNumericScale(), block->yScale()) &&
        Utils::compare_distance(m_dimY->getNumericOffset(), block->yOffset()) &&
        Utils::compare_distance(m_dimZ->getNumericScale(), block->zScale()) &&
        Utils::compare_distance(m_dimZ->getNumericOffset(), block->zOffset()))
    {
        return;
    }

    // Get the value from the buffer unscaled.  Scale the value as specified
    // in the block (the clould's scaling) and then set the value back into
    // the buffer, taking the final scaling out.
    for (PointId i = begin; i < end; ++i)
    {
        double d;
        d = buffer.getFieldAs<double>(m_dimX, i, false);
        d = d * block->xScale() + block->xOffset();
        buffer.setFieldUnscaled(m_dimX, i, d);

        d = buffer.getFieldAs<double>(m_dimY, i, false);
        d = d * block->yScale() + block->yOffset();
        buffer.setFieldUnscaled(m_dimY, i, d);

        d = buffer.getFieldAs<double>(m_dimZ, i, false);
        d = d * block->zScale() + block->zOffset();
        buffer.setFieldUnscaled(m_dimZ, i, d);
    }
}


// Read a block (set of points) from the database.
bool OciSeqIterator::readOci(Statement stmt, BlockPtr block)
{
    if (!block->fetched())
    {
        if (!stmt->Fetch())
        {
            m_atEnd = true;
            return false;
        }
        block->setFetched();
    }
    // Read the points from the blob in the row.
    readBlob(stmt, block);
    Schema *s = findSchema(stmt, block);
    block->initialize(s);
    block->clearFetched();
    return true;
}


// Each cloud may have its own scaling for X, Y and Z.  Store it away so
// that it can be applied later if necessary.
Schema* OciSeqIterator::findSchema(Statement stmt, BlockPtr block)
{
    int32_t cloudId = stmt->GetInteger(&block->pc->pc_id);
    Schema s;
    auto si = m_schemas.find(cloudId);
    if (si == m_schemas.end())
    {
        s = fetchSchema(stmt, block);
        auto i = m_schemas.insert(std::make_pair(cloudId, s));
        si = i.first;
    }
    return &(si->second);
}


pdal::Bounds<double> OciSeqIterator::getBounds(Statement stmt, BlockPtr block)
{
    double xmin, ymin, xmax, ymax;

    stmt->GetElement(&(block->blk_extent->sdo_ordinates), 0, &xmin);
    stmt->GetElement(&(block->blk_extent->sdo_ordinates), 1, &ymin);
    stmt->GetElement(&(block->blk_extent->sdo_ordinates), 2, &xmax);
    stmt->GetElement(&(block->blk_extent->sdo_ordinates), 3, &ymax);
    return pdal::Bounds<double>(xmin, ymin, xmax, ymax);
}


} // namespace sequential
} // namespace iterators

} // namespace oci
} // namespace drivers
} // namespace pdal
