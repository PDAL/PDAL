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

#include <pdal/StreamFactory.hpp>
#include <pdal/FileUtils.hpp>

#include <sstream>

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
    while (!m_streams.empty())
    {
        Set::iterator iter = m_streams.begin();
        std::istream* s = *iter;
        m_streams.erase(iter);
        assert(s);
        FileUtils::closeFile(s);
    }

    return;
}


std::istream& FilenameStreamFactory::allocate()
{
    std::istream* s = FileUtils::openFile(m_filename, true);
    if (s == NULL)
    {
        std::ostringstream oss;
        oss << "Unable to open file '" << m_filename <<"'. Check access permissions and/or directory location";
        throw pdal_error(oss.str());
    }

    m_streams.insert(s);
    return *s;
}


void FilenameStreamFactory::deallocate(std::istream& stream)
{
    Set::iterator iter = m_streams.find(&stream);
    if (iter == m_streams.end())
        throw pdal_error("incorrect stream deallocation");

    std::istream* s = *iter;
    m_streams.erase(iter);
    FileUtils::closeFile(s);

    return;
}


// --------------------------------------------------------------------

FilenameSubsetStreamFactory::StreamSet::StreamSet(const std::string& filename, boost::uint64_t offset, boost::uint64_t length)
{
    namespace io = boost::iostreams;

    std::istream* file = FileUtils::openFile(filename, true);

    Stream* source = dynamic_cast<Stream*>(file);
    assert(source!=0);

    StreamSlice* restricted_device = new StreamSlice(*source, offset, length);
    io::stream<StreamSlice>* restricted_stream = new io::stream<StreamSlice>(*restricted_device);

    m_stream = file;
    m_slice = restricted_device;
    m_streamslice = restricted_stream;

    return;
}


FilenameSubsetStreamFactory::StreamSet::~StreamSet()
{
    m_streamslice->close();
    delete m_streamslice;

    m_slice->close();
    delete m_slice;

    FileUtils::closeFile(m_stream);

    return;
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
    while (!m_streams.empty())
    {
        Map::iterator iter = m_streams.begin();
        StreamSet* set = iter->second;
        m_streams.erase(iter);
        delete set;
    }

    return;
}


std::istream& FilenameSubsetStreamFactory::allocate()
{
    StreamSet* set = new StreamSet(m_filename, m_offset, m_length);

    std::istream* stream = set->stream();

    m_streams.insert(make_pair(stream, set));

    return *stream;
}


void FilenameSubsetStreamFactory::deallocate(std::istream& stream)
{
    Map::iterator iter = m_streams.find(&stream);
    if (iter == m_streams.end())
        throw pdal_error("incorrect stream deallocation");

    StreamSet* s = iter->second;
    m_streams.erase(iter);
    delete s;

    return;
}


// --------------------------------------------------------------------


OutputStreamManager::OutputStreamManager(const std::string& filename)
    : m_isFileBased(true)
    , m_isOpen(false)
    , m_filename(filename)
    , m_ostream(NULL)
{
    return;
}


OutputStreamManager::OutputStreamManager(std::ostream* ostream)
    : m_isFileBased(false)
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

void OutputStreamManager::flush()
{
    m_ostream->flush();
}

void OutputStreamManager::open()
{
    if (m_isOpen)
        throw pdal_error("cannot re-open file or stream");

    if (m_isFileBased)
    {
        m_ostream = FileUtils::createFile(m_filename, true);
        if (m_ostream == NULL)
        {
            std::ostringstream oss;
            oss << "Unable to create file '" << m_filename <<"'. Check access permissions and/or directory location";
            throw pdal_error(oss.str());
        }
    }
    else
    {
        // nothing to do
        if (m_ostream == NULL)
            throw pdal_error("invalid stream");
    }

    m_isOpen = true;

    return;
}


void OutputStreamManager::close()
{
    if (!m_isOpen) return;

    if (m_isFileBased)
    {
        FileUtils::closeFile(m_ostream);
    }
    else
    {
        // nothing to do
    }

    m_ostream = NULL;
    m_isOpen = false;

    return;
}


std::ostream& OutputStreamManager::ostream()
{
    if (!m_isOpen || !m_ostream)
        throw pdal_error("invalid stream");
    return *m_ostream;
}


} // namespace pdal
