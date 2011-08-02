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
#include <pdal/exceptions.hpp>


namespace pdal
{


StreamManagerBase::StreamManagerBase(const std::string& filename, Type type)
    : m_isOpen(false)
    , m_type(type)
    , m_filename(filename)
{
    return;
}


const std::string& StreamManagerBase::getFileName() const
{
    return m_filename;
}


StreamManagerBase::Type StreamManagerBase::getType() const
{
    return m_type;
}


bool StreamManagerBase::isOpen() const
{
    return m_isOpen;
}

// -------------------------------------------------------------------------------

IStreamManager::IStreamManager(const std::string& filename)
    : StreamManagerBase(filename, File)
    , m_istream(NULL)
{
    return;
}


IStreamManager::IStreamManager(std::istream* istream)
    : StreamManagerBase("", Stream)
    , m_istream(istream)
{
    return;
}


IStreamManager::~IStreamManager()
{
    close();
    return;
}


void IStreamManager::open()
{
    if (m_isOpen)
        throw pdal_error("cannot re-open file or stream");

    switch (getType())
    {
    case File:
        m_istream = FileUtils::openFile(getFileName(), true);
        break;
    case Stream:
        // nothing to do
        if (m_istream == NULL)
            throw pdal_error("invalid stream");
        break;
    default:
        throw pdal_error("cannot open");
        break;
    }

    m_isOpen = true;

    return;
}


void IStreamManager::close()
{
    if (!m_isOpen) return;

    switch (getType())
    {
    case File:
        FileUtils::closeFile(m_istream);
        break;
    case Stream:
        // nothing to do
        break;
    default:
        throw pdal_error("cannot close");
        break;
    }

    m_istream = NULL;
    m_isOpen = false;

    return;
}


std::istream& IStreamManager::istream()
{
    if (!isOpen() || !m_istream)
        throw pdal_error("invalid stream");
    return *m_istream;
}


// -------------------------------------------------------------------------------


OStreamManager::OStreamManager(const std::string& filename)
    : StreamManagerBase(filename, File)
    , m_ostream(NULL)
{
    return;
}


OStreamManager::OStreamManager(std::ostream* ostream)
    : StreamManagerBase("", Stream)
    , m_ostream(ostream)
{
    return;
}


OStreamManager::~OStreamManager()
{
    close();
    return;
}


void OStreamManager::open()
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


void OStreamManager::close()
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


std::ostream& OStreamManager::ostream()
{
    if (!isOpen() || !m_ostream)
        throw pdal_error("invalid stream");
    return *m_ostream;
}


} // namespace pdal
