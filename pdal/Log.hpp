/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#pragma once

#include <cassert>
#include <memory> // shared_ptr
#include <stack>
#include <chrono>

#include <pdal/pdal_internal.hpp>
#include <pdal/util/NullOStream.hpp>

// Adapted from http://drdobbs.com/cpp/201804215

namespace pdal
{
class Log;

typedef std::shared_ptr<Log> LogPtr;

/// pdal::Log is a logging object that is provided by pdal::Stage to
/// facilitate logging operations.
class PDAL_EXPORT Log
{
private:
    /// Constructs a pdal::Log instance.
    /// @param leaderString A string to presage all log entries with
    /// @param outputName A filename or one of 'stdout', 'stdlog', or 'stderr'
    ///                   to use for outputting log information.
    /// @param timing Set to true to get timing output with log messages.
    Log(std::string const& leaderString, std::string const& outputName,
        bool timing = false);

    /// Constructs a pdal::Log instance.
    /// @param leaderString A string to presage all log entries with
    /// @param v An existing std::ostream to use for logging (instead of the
    ///          the instance creating its own)
    /// @param timing Set to true to get timing output with log messages.
    Log(std::string const& leaderString, std::ostream* v, bool timing = false);

public:
    static LogPtr makeLog(std::string const& leaderString,
        std::string const& outputName, bool timing = false);

    static LogPtr makeLog(std::string const& leaderString,
        std::ostream* v, bool timing = false);

    /** @name Destructor
    */
    /// The destructor will clean up its own internal log stream, but it will
    /// not touch one that is given via the constructor
    ~Log();

    /** @name Logging level
    */
    /// @return the logging level of the pdal::Log instance
    LogLevel getLevel()
    {
        return m_level;
    }

    /// Sets the logging level of the pdal::Log instance
    /// @param v logging level to use for get() comparison operations
    void setLevel(LogLevel v)
    {
        assert(v != LogLevel::None);
        m_level = v;
    }

    /// Set the leader string (deprecated).
    /// \param[in]  leader  Leader string.
    void setLeader(const std::string& leader)
        { pushLeader(leader); }

    /// Push the leader string onto the stack.
    /// \param  leader  Leader string
    void pushLeader(const std::string& leader)
        { m_leaders.push(leader); }

    /// Get the leader string.
    /// \return  The current leader string.
    std::string leader() const
        { return m_leaders.empty() ? std::string() : m_leaders.top(); }

    /// Pop the current leader string.
    void popLeader()
    {
        if (!m_leaders.empty())
            m_leaders.pop();
    }

    /// @return A string representing the LogLevel
    std::string getLevelString(LogLevel v) const;

    /** @name Log stream operations
    */
    /// @return the stream object that is currently being used to for log
    /// operations regardless of logging level of the instance.
    std::ostream* getLogStream()
    {
        return m_log;
    }

    /// Returns the log stream given the logging level.
    /// @param level logging level to request
    /// If the logging level asked for with
    /// pdal::Log::get is less than the logging level of the pdal::Log instance
    std::ostream& get(LogLevel level = LogLevel::Info);

    /// Sets the floating point precision
    void floatPrecision(int level);

    /// Clears the floating point precision settings of the streams
    void clearFloat();

protected:
    std::ostream *m_log;

private:
    Log(const Log&) = delete;
    Log& operator =(const Log&) = delete;
    std::string now() const;

    LogLevel m_level;
    bool m_deleteStreamOnCleanup;
    std::stack<std::string> m_leaders;
    NullOStream m_nullStream;
    bool m_timing;
    std::chrono::steady_clock m_clock;
    std::chrono::steady_clock::time_point m_start;
};

} // namespace pdal

