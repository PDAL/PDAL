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

#include <libpc/Writer.hpp>
#include <libpc/Iterator.hpp>
#include <libpc/Stage.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/exceptions.hpp>

namespace libpc
{

const boost::uint32_t Writer::s_defaultChunkSize = 1024 * 32;


Writer::Writer(Stage& prevStage)
    : m_actualNumPointsWritten(0)
    , m_targetNumPointsToWrite(0)
    , m_prevStage(prevStage)
    , m_chunkSize(s_defaultChunkSize)
{
    return;
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
    m_targetNumPointsToWrite = targetNumPointsToWrite;
    m_actualNumPointsWritten = 0;

    boost::scoped_ptr<SequentialIterator> iter(m_prevStage.createSequentialIterator());
    
    if (!iter) throw libpc_error("Unable to obtain iterator from previous stage!");

    writeBegin();

    // in case targetNumPointsToWrite is really big, we will process just one chunk at a time

    while (m_actualNumPointsWritten < m_targetNumPointsToWrite)
    {
        const boost::uint64_t numRemainingPointsToRead = m_targetNumPointsToWrite - m_actualNumPointsWritten;
        
        const boost::uint64_t numPointsToReadThisChunk64 = std::min<boost::uint64_t>(numRemainingPointsToRead, m_chunkSize);
        // this case is safe because m_chunkSize is a uint32
        const boost::uint32_t numPointsToReadThisChunk = static_cast<boost::uint32_t>(numPointsToReadThisChunk64);

        PointBuffer buffer(m_prevStage.getSchema(), numPointsToReadThisChunk);

        const boost::uint32_t numPointsReadThisChunk = iter->read(buffer);
        assert(numPointsReadThisChunk <= numPointsToReadThisChunk);

        const boost::uint32_t numPointsWrittenThisChunk = writeBuffer(buffer);
        assert(numPointsWrittenThisChunk == numPointsReadThisChunk);

        m_actualNumPointsWritten += numPointsWrittenThisChunk;

        if (iter->atEnd())
        {
            break;
        }
    }

    writeEnd();

    assert(m_actualNumPointsWritten <= m_targetNumPointsToWrite);

    return m_actualNumPointsWritten;
}


} // namespace libpc
