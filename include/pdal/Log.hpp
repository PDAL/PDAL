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

#include <pdal/pdal.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>

#include <pdal/FileUtils.hpp>
#include <iosfwd>

typedef boost::iostreams::null_sink sink;

typedef boost::iostreams::stream<sink> null_stream;

// typedef boost::shared_ptr<null_buffer> NullBufferPtr;

// Adapted from http://drdobbs.com/cpp/201804215

namespace pdal
{


class StageBase;

enum eLogLevel {
    logERROR = 0, 
    logWARNING, 
    logINFO, 
    logDEBUG, 
    logDEBUG1,
    logDEBUG2, 
    logDEBUG3, 
    logDEBUG4,
    logDEBUG5
};

class PDAL_DLL Log
{
public:
    Log(StageBase const& stage, std::string const& outputName, std::ostream* v);
    virtual ~Log();
    std::ostream& get(eLogLevel level = logINFO);
    eLogLevel getLevel() { return m_level; }
    void setLevel(eLogLevel v) { m_level = v; }
    
    std::ostream* getLogStream() { return m_log; }
    std::string getLevelString(eLogLevel v) const;
    
protected:
    std::ostream* m_log;
    null_stream m_null_stream;
    
private:
    Log(const Log&);
    Log& operator =(const Log&);
    
    StageBase const& m_stage;
    eLogLevel m_level;
    bool m_deleteStreamOnCleanup;
};



} // namespace pdal

#endif
