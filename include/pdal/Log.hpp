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

#ifndef INCLUDED_LOG_HPP
#define INCLUDED_LOG_HPP

#include <pdal/pdal_internal.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>
#include <boost/shared_ptr.hpp>
#include <pdal/FileUtils.hpp>
#include <iosfwd>

typedef boost::iostreams::null_sink sink;
typedef boost::iostreams::stream<sink> null_stream;


// Adapted from http://drdobbs.com/cpp/201804215

namespace pdal
{


/// pdal::Log is a logging object that is provided by pdal::Stage to
/// facilitate logging operations.
class PDAL_DLL Log
{
public:

    /** @name Constructors
    */
    /// Constructs a pdal::Log instance.
    /// @param leaderString A string to presage all log entries with
    /// @param outputName A filename or one of 'stdout', 'stdlog', or 'stderr'
    ///                   to use for outputting log information.
    Log(std::string const& leaderString, std::string const& outputName);

    /// Constructs a pdal::Log instance.
    /// @param leaderString A string to presage all log entries with
    /// @param v An existing std::ostream to use for logging (instead of the
    ///          the instance creating its own)
    Log(std::string const& leaderString, std::ostream* v);

    /** @name Destructor
    */
    /// The destructor will clean up its own internal log stream, but it will
    /// not touch one that is given via the constructor
    ~Log();

    /** @name Logging level
    */
    /// @return the logging level of the pdal::Log instance
    LogLevel::Enum getLevel()
    {
        return m_level;
    }

    /// Sets the logging level of the pdal::Log instance
    /// @param v logging level to use for get() comparison operations
    void setLevel(LogLevel::Enum v)
    {
        m_level = v;
    }

    /// @return A string representing the LogLevel
    std::string getLevelString(LogLevel::Enum v) const;

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
    /// an ostream with a boost::iostreams::null_sink is returned.
    std::ostream& get(LogLevel::Enum level = LogLevel::Info);

    /// Sets the floating point precision
    void floatPrecision(int level);

    /// Clears the floating point precision settings of the streams
    void clearFloat();

    /** @name private attributes
    */

protected:
    std::ostream* m_log;
    null_stream m_null_stream;

private:
    Log(const Log&);
    Log& operator =(const Log&);

    LogLevel::Enum m_level;
    bool m_deleteStreamOnCleanup;
    std::string m_leader;
};

typedef boost::shared_ptr<Log> LogPtr;


} // namespace pdal

#endif
