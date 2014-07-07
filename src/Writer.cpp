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

#include <boost/scoped_ptr.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/UserCallback.hpp>

#include <pdal/PipelineWriter.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{


Writer::Writer(const Options& options) : Stage(options),
    m_callback(new UserCallback), m_writer_buffer(0)
{}


Writer::Writer() : m_callback(new UserCallback), m_writer_buffer(0)
{}


Writer::~Writer()
{
    delete m_writer_buffer;
}

const SpatialReference& Writer::getSpatialReference() const
{
    return m_spatialReference;
}


void Writer::setSpatialReference(const SpatialReference& srs)
{
    m_spatialReference = srs;
}


void Writer::setUserCallback(UserCallback* userCallback)
{
    m_callback.reset(userCallback);
}


boost::uint64_t Writer::write(boost::uint64_t targetNumPointsToWrite,
                              boost::uint64_t startingPosition,
                              boost::uint64_t chunkSize)
{
    if (!isInitialized())
    {
        throw pdal_error("stage not initialized");
    }

    boost::uint64_t actualNumPointsWritten = 0;

    const Schema& schema = getPrevStage().getSchema();

    if (m_writer_buffer == 0)
    {
        boost::uint64_t capacity(targetNumPointsToWrite);
        if (capacity == 0)
        {
            if (chunkSize)
                capacity = chunkSize;
            else
                capacity = 131072;
        }
        else
        {
            if (chunkSize)
                capacity = (std::min)(static_cast<boost::uint64_t>(chunkSize), targetNumPointsToWrite) ;
            else
                capacity = targetNumPointsToWrite;
        }

        if (capacity > std::numeric_limits<boost::uint32_t>::max())
            throw pdal_error("Buffer capacity is larger than 2^32 points!");
        m_writer_buffer = new PointBuffer(schema, static_cast<boost::uint32_t>(capacity));

    }

    boost::scoped_ptr<StageSequentialIterator> iter(getPrevStage().createSequentialIterator(*m_writer_buffer));

    if (startingPosition)
        iter->skip(startingPosition);

    if (!iter) throw pdal_error("Unable to obtain iterator from previous stage!");

    // if we don't have an SRS, try to forward the one from the prev stage
    if (m_spatialReference.empty()) m_spatialReference = getPrevStage().getSpatialReference();

    writeBegin(targetNumPointsToWrite);

    iter->readBegin();



    //
    // The user has requested a specific number of points: proceed a
    // chunk at a time until we reach that number.  (If that number
    // is 0, we proceed until no more points can be read.)
    //
    // If the user requests an interrupt while we're running, we'll throw.
    //
    while (true)
    {
        // have we hit the end already?
        if (iter->atEnd()) break;

        // rebuild our PointBuffer, if it needs to hold less than the default max chunk size
        if (targetNumPointsToWrite != 0)
        {
            const boost::int64_t numRemainingPointsToRead = targetNumPointsToWrite - actualNumPointsWritten;

            const boost::int64_t numPointsToReadThisChunk64 = std::min<boost::int64_t>(numRemainingPointsToRead, chunkSize == 0 ? numRemainingPointsToRead : chunkSize);
            const boost::uint32_t numPointsToReadThisChunk = static_cast<boost::uint32_t>(numPointsToReadThisChunk64);

            // we are reusing the buffer, so we may need to adjust the capacity for the last (and likely undersized) chunk
            if (m_writer_buffer->getCapacity() != numPointsToReadThisChunk)
            {
                m_writer_buffer->resize(numPointsToReadThisChunk);
            }
        }

        // read...
        iter->readBufferBegin(*m_writer_buffer);
        const boost::uint32_t numPointsReadThisChunk = iter->readBuffer(*m_writer_buffer);
        iter->readBufferEnd(*m_writer_buffer);

        assert(numPointsReadThisChunk == m_writer_buffer->getNumPoints());
        assert(numPointsReadThisChunk <= m_writer_buffer->getCapacity());
        
        // Some drivers may do header setups and such, even for situations 
        // where there's no points, so we should a buffer begin. 
        writeBufferBegin(*m_writer_buffer);
        
        // were there no points left to write this chunk?
        if (numPointsReadThisChunk == 0) 
        {
            // Match the above writeBufferBegin now that 
            // we're breaking the loop
            writeBufferEnd(*m_writer_buffer);
            break;
        }

        // write...
        const boost::uint32_t numPointsWrittenThisChunk = writeBuffer(*m_writer_buffer);
        assert(numPointsWrittenThisChunk == numPointsReadThisChunk);
        writeBufferEnd(*m_writer_buffer);

        // update count
        actualNumPointsWritten += numPointsWrittenThisChunk;

        if (targetNumPointsToWrite != 0)
        {
            // have we done enough yet?
            if (actualNumPointsWritten >= targetNumPointsToWrite) break;
        }

        // reset the buffer, so we can use it again
        m_writer_buffer->setNumPoints(0);
    }

    iter->readEnd();

    writeEnd(actualNumPointsWritten);

    assert((targetNumPointsToWrite == 0) || (actualNumPointsWritten <= targetNumPointsToWrite));

    return actualNumPointsWritten;
}


boost::property_tree::ptree Writer::serializePipeline() const
{
    boost::property_tree::ptree tree;

    tree.add("<xmlattr>.type", getName());

    PipelineWriter::write_option_ptree(tree, getOptions());

    const Stage& stage = getPrevStage();
    boost::property_tree::ptree subtree = stage.serializePipeline();

    tree.add_child(subtree.begin()->first, subtree.begin()->second);

    boost::property_tree::ptree root;
    root.add_child("Writer", tree);

    return root;
}


} // namespace pdal
