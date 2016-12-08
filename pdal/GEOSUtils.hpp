/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <memory>

#include <geos_c.h>

#include <pdal/pdal_types.hpp>
#include <pdal/Log.hpp>

namespace pdal
{

namespace geos
{

class PDAL_DLL ErrorHandler
{
public:
    ~ErrorHandler();

    /**
      Get the singleton error handler.

      \return  Reference to the error handler.
    */
    static ErrorHandler& get();

    /**
      Set the log and debug state of the error handler.  This is a convenience
      and is equivalent to calling setLog() and setDebug().

      \param log  Log to write to.
      \param doDebug  Debug state of the error handler.
    */
    void set(LogPtr log, bool doDebug);

    /**
      Set the log to which error/debug messages should be written.

      \param log  Log to write to.
    */
    void setLog(LogPtr log);

    /**
      Set the debug state of the error handler.  If the error handler is set
      to debug, output is logged instead of causing an exception.

      \param debug  The debug state of the error handler.
    */
    void setDebug(bool debug);

    /**
      Get the GEOS context handle.

      \return  The GEOS context handle.
    */
    GEOSContextHandle_t ctx() const;

private:
    ErrorHandler();

    void handle(const char *msg, bool notice);
    static void vaErrorCb(const char *msg, ...);
    static void vaNoticeCb(const char *msg, ...);

    GEOSContextHandle_t m_ctx;
    bool m_debug;
    LogPtr m_log;
    static std::unique_ptr<ErrorHandler> m_instance;
};

} // namespace geos
} // namespace pdal

