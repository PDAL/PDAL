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

#include <pdal/StreamOwner.hpp>
#include <pdal/Utils.hpp>
#include <pdal/exceptions.hpp>


namespace pdal
{

StreamOwner::StreamOwner(const std::string filename, Mode mode)
    : m_mode(mode)
    , m_owned(true)
    , m_istream(NULL)
    , m_ostream(NULL)
{
    if (m_mode == ReadMode)
    {
        m_istream = Utils::openFile(filename);
    }
    else if (m_mode == WriteMode)
    {
        m_ostream = Utils::createFile(filename);
    }
    else
    {
        throw pdal_error("invalid file mode");
    }

    return;
}


StreamOwner::StreamOwner(std::istream* istream)
    : m_mode(ReadMode)
    , m_owned(false)
    , m_istream(istream)
    , m_ostream(NULL)
{
    return;
}


StreamOwner::StreamOwner(std::ostream* ostream)
    : m_mode(WriteMode)
    , m_owned(false)
    , m_istream(NULL)
    , m_ostream(ostream)
{
    return;
}


StreamOwner::~StreamOwner()
{
    if (m_istream)
    {
        if (m_owned)
        {
            Utils::closeFile(m_istream);
        }
        m_istream = NULL;
    }
    if (m_ostream)
    {
        if (m_owned)
        {
            Utils::closeFile(m_ostream);
        }
        m_ostream = NULL;
    }

    return;
}


std::istream* StreamOwner::istream()
{
    if (m_mode != ReadMode)
    {
        throw pdal_error("invalid file mode");
    }
    return m_istream;
}


std::ostream* StreamOwner::ostream()
{
    if (m_mode != WriteMode)
    {
        throw pdal_error("invalid file mode");
    }
    return m_ostream;
}


} // namespace pdal
