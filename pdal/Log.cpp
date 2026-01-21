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
#include <pdal/PDALUtils.hpp>

#include <fstream>
#include <ostream>

namespace pdal
{

Log::Log(std::string const& leaderString, std::string const& outputName,
        bool timing)
    : m_level(LogLevel::Warning)
    , m_deleteStreamOnCleanup(false)
    , m_timing(timing)
{
    if (Utils::iequals(outputName, "stdlog"))
        m_log = &std::clog;
    else if (Utils::iequals(outputName, "stderr"))
        m_log = &std::cerr;
    else if (Utils::iequals(outputName, "stdout"))
        m_log = &std::cout;
    else if (Utils::iequals(outputName, "devnull"))
        m_log = &m_nullStream;
    else
    {
        m_log = Utils::createFile(outputName);
        m_deleteStreamOnCleanup = true;
    }
    m_leaders.push(leaderString);
    if (m_timing)
        m_start = m_clock.now();
}


Log::Log(std::string const& leaderString, std::ostream* v, bool timing)
    : m_level(LogLevel::Error)
    , m_deleteStreamOnCleanup(false)
    , m_timing(timing)
{
    m_log = v;
    m_leaders.push(leaderString);
    if (m_timing)
        m_start = m_clock.now();
}

LogPtr Log::makeLog(std::string const& leaderString,
    std::string const& outputName, bool timing)
{
    return LogPtr(new Log(leaderString, outputName, timing));
}


LogPtr Log::makeLog(std::string const& leaderString, std::ostream* v,
    bool timing)
{
    return LogPtr(new Log(leaderString, v, timing));
}


Log::~Log()
{
    if (m_deleteStreamOnCleanup)
    {
        m_log->flush();
        delete m_log;
    }
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


std::ostream& Log::get(LogLevel level)
{
    const auto incoming(Utils::toNative(level));
    const auto stored(Utils::toNative(m_level));
    const auto nativeDebug(Utils::toNative(LogLevel::Debug));
    if (incoming <= stored)
    {
        const std::string l = leader();

        *m_log << "(" << l;
         if (l.size())
             *m_log << " ";
         *m_log << getLevelString(level);
         if (m_timing)
             *m_log << " " << now();
         *m_log <<") " <<
         std::string(incoming < nativeDebug ? 0 : incoming - nativeDebug,
             '\t');
        return *m_log;
    }
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_nullStream;
}


std::string Log::getLevelString(LogLevel level) const
{
    switch (level)
    {
        case LogLevel::Error:
            return "Error";
            break;
        case LogLevel::Warning:
            return "Warning";
            break;
        case LogLevel::Info:
            return "Info";
            break;
        default:
            return "Debug";
    }
}

std::string Log::now() const
{
    std::chrono::steady_clock::time_point end = m_clock.now();

    std::chrono::duration<double> diff = end - m_start;
    std::stringstream ss;

    ss << std::fixed << std::setprecision(3) << diff.count();
    return ss.str();
}

} // namespace
