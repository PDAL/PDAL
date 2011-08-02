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

//
#include <boost/scoped_ptr.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/exceptions.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{

const boost::uint32_t Writer::s_defaultChunkSize = 1024 * 32;


Writer::Writer(Stage& prevStage, const Options& options)
    : StageBase(options)
    , m_actualNumPointsWritten(0)
    , m_targetNumPointsToWrite(0)
    , m_prevStage(prevStage)
    , m_chunkSize(s_defaultChunkSize)

{
    return;
}


void Writer::initialize()
{
    getPrevStage().initialize();

    StageBase::initialize();

    return;
}


const Stage& Writer::getPrevStage() const
{
    return m_prevStage;
}


Stage& Writer::getPrevStage()
{
    return m_prevStage;
}


void Writer::setChunkSize(boost::uint32_t chunkSize)
{
    m_chunkSize = chunkSize;
}


boost::uint32_t Writer::getChunkSize() const
{
    return m_chunkSize;
}


boost::uint64_t Writer::write(boost::uint64_t targetNumPointsToWrite)
{
    if (!isInitialized())
    {
        throw pdal_error("stage not initialized");
    }

    m_targetNumPointsToWrite = targetNumPointsToWrite;
    m_actualNumPointsWritten = 0;
         
    boost::scoped_ptr<StageSequentialIterator> iter(m_prevStage.createSequentialIterator());
    
    if (!iter) throw pdal_error("Unable to obtain iterator from previous stage!");

    writeBegin();

    iter->readBegin();

    if (m_targetNumPointsToWrite == 0)
    {
        //
        // user has requested 0 points, which means "as many as is available":
        // proceed a chunk at a time until we reach the end
        //
        PointBuffer buffer(m_prevStage.getSchema(), m_chunkSize);

        while (true)
        {
            // have we done enough yet?
            if (iter->atEnd()) break;

            // read...
            iter->readBufferBegin(buffer);
            const boost::uint32_t numPointsReadThisChunk = iter->readBuffer(buffer);
            iter->readBufferEnd(buffer);

            // have we reached the end yet?
            if (numPointsReadThisChunk == 0) break;
        
            // write...
            writeBufferBegin(buffer);
            const boost::uint32_t numPointsWrittenThisChunk = writeBuffer(buffer);
            writeBufferEnd(buffer);

            // update count
            m_actualNumPointsWritten += numPointsWrittenThisChunk;

            // reset the buffer, so we can use it again
            buffer.setNumPoints(0);
        }
    }
    else 
    {
        //
        // user has requested a specific number of points:
        // proceed a chunk at a time until we reach that number
        //
        PointBuffer buffer(m_prevStage.getSchema(), m_chunkSize);
        while (true)
        {
            // have we done enough yet?
            if (iter->atEnd()) break;

            const boost::uint64_t numRemainingPointsToRead = m_targetNumPointsToWrite - m_actualNumPointsWritten;
        
            const boost::uint64_t numPointsToReadThisChunk64 = std::min<boost::uint64_t>(numRemainingPointsToRead, m_chunkSize);
            // this case is safe because m_chunkSize is a uint32
            const boost::uint32_t numPointsToReadThisChunk = static_cast<boost::uint32_t>(numPointsToReadThisChunk64);

            // we are reusing the buffer, so we may need to adjust the capacity for the last (and likely undersized) chunk
            if (buffer.getCapacity() != numPointsToReadThisChunk)
            {
                buffer = PointBuffer(m_prevStage.getSchema(), numPointsToReadThisChunk);
            }

            // read...
            iter->readBufferBegin(buffer);
            const boost::uint32_t numPointsReadThisChunk = iter->readBuffer(buffer);
            iter->readBufferEnd(buffer);

            assert(numPointsReadThisChunk <= numPointsToReadThisChunk);

            // have we reached the end yet?
            if (numPointsReadThisChunk == 0) break;

            // write...
            writeBufferBegin(buffer);
            const boost::uint32_t numPointsWrittenThisChunk = writeBuffer(buffer);
            assert(numPointsWrittenThisChunk == numPointsReadThisChunk);
            writeBufferEnd(buffer);

            // update count
            m_actualNumPointsWritten += numPointsWrittenThisChunk;

            // have we done enough yet?
            if (m_actualNumPointsWritten >= m_targetNumPointsToWrite) break;
        }
    }

    iter->readEnd();

    writeEnd();

    // assert(m_actualNumPointsWritten <= m_targetNumPointsToWrite);

    return m_actualNumPointsWritten;
}


boost::property_tree::ptree Writer::generatePTree() const
{
    boost::property_tree::ptree tree;

    tree.add("Type", getName());

    boost::property_tree::ptree optiontree = getOptions().getPTree();
    tree.add_child(optiontree.begin()->first, optiontree.begin()->second);

    const Stage& stage = getPrevStage();
    boost::property_tree::ptree subtree = stage.generatePTree();

    tree.add_child(subtree.begin()->first, subtree.begin()->second);
    
    boost::property_tree::ptree root;
    root.add_child("Writer", tree);

    return root;
}


} // namespace pdal
