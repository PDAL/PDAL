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


StreamOwnerBase::StreamOwnerBase(const std::string& filename)
    : m_filename(filename)
{
    return;
}


const std::string& StreamOwnerBase::getFileName() const
{
    return m_filename;
}


// -------------------------------------------------------------------------------

IStreamOwner::IStreamOwner(const std::string& filename)
    : StreamOwnerBase(filename)
    , m_pistream(Utils::openFile(filename, true))
    , m_istream(*m_pistream)
{
    return;
}


IStreamOwner::IStreamOwner(std::istream& istream)
    : StreamOwnerBase("")
    , m_pistream(NULL)
    , m_istream(istream)
{
    return;
}


IStreamOwner::~IStreamOwner()
{
    if (m_pistream)
    {
        Utils::closeFile(m_pistream);
        m_pistream = NULL;
    }

    return;
}


std::istream& IStreamOwner::istream()
{
    return m_istream;
}


// -------------------------------------------------------------------------------


OStreamOwner::OStreamOwner(const std::string& filename)
    : StreamOwnerBase(filename)
    , m_postream(Utils::createFile(filename, true))
    , m_ostream(*m_postream)
{
    return;
}


OStreamOwner::OStreamOwner(std::ostream& ostream)
    : StreamOwnerBase("")
    , m_postream(NULL)
    , m_ostream(ostream)
{
    return;
}


OStreamOwner::~OStreamOwner()
{
    if (m_postream)
    {
        Utils::closeFile(m_postream);
        m_postream = NULL;
    }

    return;
}


std::ostream& OStreamOwner::ostream()
{
    return m_ostream;
}


} // namespace pdal
