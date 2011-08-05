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

#include <pdal/drivers/las/Iterator.hpp>

#include <iostream>

#include "ZipPoint.hpp"
#include <laszip/lasunzipper.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/exceptions.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal { namespace drivers { namespace las {


IteratorBase::IteratorBase(const LasReader& reader)
    : m_reader(reader)
    , m_istream(NULL)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());

    if (m_reader.isCompressed())
    {
#ifdef PDAL_HAVE_LASZIP
        initializeZip();
#else
        throw pdal_error("LASzip is not enabled for this pdal::drivers::las::IteratorBase!");
#endif
    }

    return;
}


IteratorBase::~IteratorBase()
{
#ifdef PDAL_HAVE_LASZIP
    m_zipPoint.reset();
    m_unzipper.reset();
#endif
    FileUtils::closeFile(m_istream);
}


void IteratorBase::initializeZip()
{
#ifdef PDAL_HAVE_LASZIP
    if (!m_zipPoint)
    {
        PointFormat format = m_reader.getPointFormat();
        boost::scoped_ptr<ZipPoint> z(new ZipPoint(format, m_reader.getVLRs()));
        m_zipPoint.swap(z);
    }

    if (!m_unzipper)
    {
        boost::scoped_ptr<LASunzipper> z(new LASunzipper());
        m_unzipper.swap(z);

        bool stat(false);
        m_istream->seekg(m_reader.getPointDataOffset(), std::ios::beg);
        stat = m_unzipper->open(*m_istream, m_zipPoint->GetZipper());

        // Martin moves the stream on us 
        m_zipReadStartPosition = m_istream->tellg();
        if (!stat)
        {
            std::ostringstream oss;
            const char* err = m_unzipper->get_error();
            if (err==NULL) err="(unknown error)";
            oss << "Failed to open LASzip stream: " << std::string(err);
            throw pdal_error(oss.str());
        }
    }
#endif
    return;
}


SequentialIterator::SequentialIterator(const LasReader& reader)
    : IteratorBase(reader)
    , pdal::ReaderSequentialIterator(reader)
{
    return;
}


SequentialIterator::~SequentialIterator()
{
    return;
}


static inline boost::uint32_t safeconvert64to32(boost::uint64_t x64) // BUG: move this to Utils header?
{
    if (x64 > std::numeric_limits<boost::uint32_t>::max())
    {
        throw pdal_error("cannot support seek offsets greater than 32-bits");
    }

    const boost::uint32_t x32 = static_cast<boost::uint32_t>(x64);
    return x32;
}


boost::uint64_t SequentialIterator::skipImpl(boost::uint64_t count)
{
#ifdef PDAL_HAVE_LASZIP
    if (m_unzipper)
    {
        const boost::uint32_t pos32 = safeconvert64to32(getIndex() + count);
        m_unzipper->seek(pos32);
    }
    else
    {
        boost::uint64_t delta = Support::getPointDataSize(m_reader.getPointFormat());
        m_istream->seekg(delta * count, std::ios::cur);
    }
#else
        boost::uint64_t delta = Support::getPointDataSize(m_reader.getPointFormat());
        m_istream->seekg(delta * count, std::ios::cur);
#endif
    return count;
}


bool SequentialIterator::atEndImpl() const
{
    return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t SequentialIterator::readBufferImpl(PointBuffer& data)
{
#ifdef PDAL_HAVE_LASZIP
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex(), m_unzipper.get(), m_zipPoint.get());
#else
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex(), NULL, NULL);

#endif
}



RandomIterator::RandomIterator(const LasReader& reader)
    : IteratorBase(reader)
    , pdal::ReaderRandomIterator(reader)
{
    return;
}


RandomIterator::~RandomIterator()
{
    return;
}


boost::uint64_t RandomIterator::seekImpl(boost::uint64_t count)
{
#ifdef PDAL_HAVE_LASZIP
    if (m_unzipper)
    {
        const boost::uint32_t pos32 = safeconvert64to32(count);
        m_unzipper->seek(pos32);
    }
    else
    {
        boost::uint64_t delta = Support::getPointDataSize(m_reader.getPointFormat());
        m_istream->seekg(m_reader.getPointDataOffset() + delta * count);
    }
#else

    boost::uint64_t delta = Support::getPointDataSize(m_reader.getPointFormat());
    m_istream->seekg(m_reader.getPointDataOffset() + delta * count);

#endif

    return count;
}


boost::uint32_t RandomIterator::readBufferImpl(PointBuffer& data)
{
#ifdef PDAL_HAVE_LASZIP
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex(), m_unzipper.get(), m_zipPoint.get());
#else
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex(), NULL, NULL);

#endif
}


} } } // namespaces
