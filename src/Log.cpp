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

#include <pdal/Log.hpp>
#include <boost/algorithm/string.hpp>

#include <ostream>
#include <sstream>

namespace pdal
{

using namespace LogLevel;

Log::Log(std::string const& leaderString,
         std::string const& outputName)
    : m_level(Error)
    , m_deleteStreamOnCleanup(false)
    , m_leader(leaderString)
{
    if (boost::iequals(outputName, "stdlog"))
    {
        m_log = &std::clog;
    }
    else if (boost::iequals(outputName, "stderr"))
    {
        m_log = &std::cerr;
    }
    else if (boost::iequals(outputName, "stdout"))
    {
        m_log = &std::cout;
    }
    else
    {
        m_log = FileUtils::createFile(outputName);
        m_deleteStreamOnCleanup = true;
    }

    m_null_stream.open(sink());
}


Log::Log(std::string const& leaderString,
         std::ostream* v)
    : m_level(Error)
    , m_deleteStreamOnCleanup(false)
    , m_leader(leaderString)
{
    m_log = v;

    m_null_stream.open(sink());
}


Log::~Log()
{

    if (m_deleteStreamOnCleanup)
    {
        m_log->flush();
        delete m_log;
    }

    m_log = 0;
}

void Log::floatPrecision(int level)
{
    m_log->setf(std::ios_base::fixed, std::ios_base::floatfield);
    m_log->precision(level);
}

void Log::clearFloat()
{
    m_log->unsetf(std::ios_base::fixed);
    m_log->unsetf(std::ios_base::floatfield);
}

std::ostream& Log::get(LogLevel::Enum level)
{
    if (level <= m_level)
    {
        *m_log << "(" << m_leader << " "<< getLevelString(level) <<": " << level << "): ";
        *m_log << std::string(level < Debug ? 0 : level - Debug, '\t');
        return *m_log;
    }
    else
    {
        return m_null_stream;
    }

}

std::string Log::getLevelString(LogLevel::Enum level) const
{
    std::ostringstream output;

    switch (level)
    {
        case Error:
            output << "Error";
            break;
        case Warning:
            output << "Warning";
            break;
        case Info:
            output << "Info";
            break;
        default:
            output << "Debug";
    }

    return output.str();
}
} // namespace
