/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include <cpl_conv.h>
#include <cpl_error.h>

namespace pdal
{
namespace gdal
{

// This is a little confusing because we have a singleton error handler with
// a single log pointer, but we set the log pointer/debug state as if we
// were taking advantage of GDAL's thread-specific error handing.
//
// We lock the log/debug so that it doesn't
// get changed while another thread is using or setting.
class PDAL_DLL ErrorHandler
{
public:
    ErrorHandler();
    ~ErrorHandler();

    /**
      Get the singleton error handler.

      \return  Reference to the error handler.
    */
    static ErrorHandler& getGlobalErrorHandler();

    /**
      Set the log and debug state of the error handler.  This is
      a convenience and is equivalent to calling setLog() and setDebug().

      \param log  Log to write to.
      \param doDebug  Debug state of the error handler.
    */
    void set(LogPtr log, bool doDebug);


    /**
      Clear an error handler.
    */
    void clear();

    /**
      Set the log to which error/debug messages should be written.

      \param log  Log to write to.
    */
    void setLog(LogPtr log);

    /**
      Set the debug state of the error handler.  Setting to true will also
      set the environment variable CPL_DEBUG to "ON".  This will force GDAL
      to emit debug error messages which will be logged by this handler.

      \param doDebug  Whether we're setting or clearing the debug state.
    */
    void setDebug(bool doDebug);

    /**
      Get the last error and clear the error last error value.

      \return  The last error number.
    */
    int errorNum();

    /**
      Handle error messages from GDAL.
    */
    void handle(int level, int num, const char *msg);

private:
    std::mutex m_mutex;
    bool m_debug;
    pdal::LogPtr m_log;
    mutable int m_errorNum;
    bool m_cplSet;
    CPLErrorHandler m_prevHandler;
};

class ErrorHandlerSuspender
{
public:
    ErrorHandlerSuspender();
    ~ErrorHandlerSuspender();
};

} // namespace gdal
} // namespace pdal
