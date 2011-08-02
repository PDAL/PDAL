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

#include <pdal/StageBase.hpp>

#include <iostream>

#include <pdal/exceptions.hpp>


namespace pdal
{


StageBase::StageBase(const Options& options)
    : m_initialized(false)
    , m_options(options)
    , m_debug(options.getValueOrDefault<bool>("debug", false))
    , m_verbose(options.getValueOrDefault<boost::uint8_t>("verbose", 0))
{
    return;
}


StageBase::~StageBase()
{
    return;
}


void StageBase::initialize()
{
    // it is illegal to call initialize() twice
    if (m_initialized)
    {
        throw pdal_error("Class already initialized: " + this->getName());
    }

    m_debug = m_options.getValueOrDefault<bool>("debug", false);
    m_verbose = m_options.getValueOrDefault<boost::uint8_t>("verbose", 0);

    m_initialized = true;

    return;
}


bool StageBase::isInitialized() const
{
    return m_initialized;
}


const Options& StageBase::getOptions() const
{
    return m_options;
}


Options& StageBase::getOptions()
{
    return m_options;
}


bool StageBase::isDebug() const
{
    return m_debug;
}


bool StageBase::isVerbose() const
{   
    return m_verbose>0;
}


boost::uint8_t StageBase::getVerboseLevel() const
{
    return m_verbose;
}


std::ostream& operator<<(std::ostream& ostr, const StageBase& stage)
{
    ostr << "  Name: " << stage.getName() << std::endl;
    ostr << "  Description: " << stage.getDescription() << std::endl;

    return ostr;
}


} // namespace pdal
