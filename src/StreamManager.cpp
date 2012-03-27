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

#include <pdal/StreamManager.hpp>
#include <pdal/FileUtils.hpp>

namespace pdal
{

// --------------------------------------------------------------------

PassthruStreamFactory::PassthruStreamFactory(std::istream& s)
    : StreamFactory()
    , m_istream(s)
    , m_allocated(false)
{
    return;
}

PassthruStreamFactory::~PassthruStreamFactory()
{
    return;
}

std::istream& PassthruStreamFactory::allocate()
{
    if (m_allocated)
        throw pdal_error("can't allocate additional stream");

    m_allocated = true;
    return m_istream;
}


void PassthruStreamFactory::deallocate(std::istream& stream)
{
    if (!m_allocated)
        throw pdal_error("incorrect stream deallocation");

    if (&stream != &m_istream)
        throw pdal_error("incorrect stream deallocation");

    m_allocated = false;
}


// --------------------------------------------------------------------

FilenameStreamFactory::FilenameStreamFactory(const std::string& name)
    : StreamFactory()
    , m_filename(name)
{
    return;
}

FilenameStreamFactory::~FilenameStreamFactory()
{
    while (m_istreams.size())
    {
        std::istream* s = m_istreams.back();
        m_istreams.pop_back();
        if (s)
        {
            FileUtils::closeFile(s);
        }
    }

    return;
}


std::istream& FilenameStreamFactory::allocate()
{
    std::istream* s = FileUtils::openFile(m_filename, true);
    m_istreams.push_back(s);
    return *s;
}


void FilenameStreamFactory::deallocate(std::istream& stream)
{
    for (unsigned int i=0; i<m_istreams.size(); i++)
    {
        if (m_istreams[i] == &stream)
        {
            FileUtils::closeFile(m_istreams[i]);
            m_istreams[i] = NULL;
            return;
        }
    }

    throw pdal_error("incorrect stream deallocation");
}


// --------------------------------------------------------------------

FilenameSubsetStreamFactory::StreamSet::StreamSet(std::istream* a, FStreamSlice* b, FStreamSliceStream* c)
    : stream(a)
    , slice(b)
    , streamslice(c)
{
    return;
}


FilenameSubsetStreamFactory::StreamSet::~StreamSet()
{
    streamslice->close();
    delete streamslice;

    slice->close();
    delete slice;

    FileUtils::closeFile(stream);

    return;
}


bool FilenameSubsetStreamFactory::StreamSet::match(std::istream& s) const
{
    std::istream* ss(&s);
    return ss == streamslice;
}


FilenameSubsetStreamFactory::FilenameSubsetStreamFactory(const std::string& name, boost::uint64_t offset, boost::uint64_t length)
    : StreamFactory()
    , m_filename(name)
    , m_offset(offset)
    , m_length(length)
{
    return;
}

FilenameSubsetStreamFactory::~FilenameSubsetStreamFactory()
{
    namespace io = boost::iostreams;

    for (unsigned int i=0; i<m_streams.size(); i++)
    {
        StreamSet* set = m_streams[i];
        if (set)
        {
            delete set;
            m_streams[i] = NULL;
        }
    }

    return;
}


std::istream& FilenameSubsetStreamFactory::allocate()
{
    namespace io = boost::iostreams;

    std::istream* file = FileUtils::openFile(m_filename, true);

    FStream* source = dynamic_cast<FStream*>(file);
    assert(source!=0);
    
    FStreamSlice* restricted_device = new FStreamSlice(*source, m_offset, m_length);
    io::stream<FStreamSlice>* restricted_stream = new io::stream<FStreamSlice>(*restricted_device);

    StreamSet* set = new StreamSet(file, restricted_device, restricted_stream);
    m_streams.push_back(set);

    return *restricted_stream;
}


void FilenameSubsetStreamFactory::deallocate(std::istream& stream)
{
    for (unsigned int i=0; i<m_streams.size(); i++)
    {
        StreamSet* set = m_streams[i];
        if (set && set->match(stream))
        {
            delete set;
            m_streams[i] = NULL;
            return;
        }
    }

    throw pdal_error("incorrect stream deallocation");
}


// --------------------------------------------------------------------


OutputStreamManager::OutputStreamManager(const std::string& filename)
    : m_type(File)
    , m_isOpen(false)
    , m_filename(filename)
    , m_ostream(NULL)
{
    return;
}


OutputStreamManager::OutputStreamManager(std::ostream* ostream)
    : m_type(Stream)
    , m_isOpen(false)
    , m_filename("")
    , m_ostream(ostream)
{
    return;
}


OutputStreamManager::~OutputStreamManager()
{
    close();
    return;
}


const std::string& OutputStreamManager::getFileName() const
{
    return m_filename;
}


OutputStreamManager::Type OutputStreamManager::getType() const
{
    return m_type;
}


bool OutputStreamManager::isOpen() const
{
    return m_isOpen;
}


void OutputStreamManager::open()
{
    if (m_isOpen)
        throw pdal_error("cannot re-open file or stream");

    switch (getType())
    {
    case File:
        m_ostream = FileUtils::createFile(getFileName(), true);
        break;
    case Stream:
        // nothing to do
        if (m_ostream == NULL)
            throw pdal_error("invalid stream");
        break;
    default:
        throw pdal_error("cannot open");
        break;
    }

    m_isOpen = true;

    return;
}


void OutputStreamManager::close()
{
    if (!m_isOpen) return;

    switch (getType())
    {
    case File:
        FileUtils::closeFile(m_ostream);
        break;
    case Stream:
        // nothing to do
        break;
    default:
        throw pdal_error("cannot close");
        break;
    }

    m_ostream = NULL;
    m_isOpen = false;

    return;
}


std::ostream& OutputStreamManager::ostream()
{
    if (!isOpen() || !m_ostream)
        throw pdal_error("invalid stream");
    return *m_ostream;
}


} // namespace pdal
