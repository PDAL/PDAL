/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include <pdal/GEOSUtils.hpp>
#include <pdal/Log.hpp>

#include <cstdarg>
#include <functional>
#include <map>
#include <sstream>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace geos
{

std::unique_ptr<ErrorHandler> ErrorHandler::m_instance;

// Allocating here instead of just making a static because I'm not really
// sure what GEOS_init_r does and when it can be called.  It doesn't hurt
// anything.
ErrorHandler& ErrorHandler::get()
{
    if (!m_instance)
        m_instance.reset(new ErrorHandler);
    return *m_instance;
}


void ErrorHandler::vaErrorCb(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), msg, args);
    ErrorHandler::get().handle(buf, false);
    va_end(args);
}


void ErrorHandler::vaNoticeCb(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), msg, args);
    ErrorHandler::get().handle(buf, true);
    va_end(args);
}


ErrorHandler::ErrorHandler() : m_debug(false)
{
// #ifdef GEOS_init_r

#if (GEOS_CAPI_VERSION_MINOR >= 9)

    m_ctx = GEOS_init_r();

    auto errorCb = [](const char *msg, void *userData)
    {
        get().handle(msg, false);
    };
    GEOSContext_setErrorMessageHandler_r(m_ctx, errorCb, NULL);

    auto noticeCb = [](const char *msg, void *userData)
    {
        get().handle(msg, true);
    };
    GEOSContext_setNoticeMessageHandler_r(m_ctx, noticeCb, NULL);
#else
    m_ctx = initGEOS_r(NULL, NULL);
    GEOSContext_setErrorHandler_r(m_ctx, vaErrorCb);
    GEOSContext_setNoticeHandler_r(m_ctx, vaNoticeCb);
#endif
}


ErrorHandler::~ErrorHandler()
{
#ifdef GEOS_finish_r
    GEOS_finish_r(m_ctx);
#else
    finishGEOS_r(m_ctx);
#endif
}


void ErrorHandler::set(LogPtr log, bool debug)
{
    setLog(log);
    setDebug(debug);
}


void ErrorHandler::setDebug(bool debug)
{
    m_debug = debug;
}


void ErrorHandler::setLog(LogPtr log)
{
    m_log = log;
}


void ErrorHandler::handle(const char *msg, bool notice)
{
    std::ostringstream oss;
    if (!notice)
    {
        oss << "GEOS failure: '" << msg << "'";
        throw pdal_error(oss.str());
    }
    else if (m_debug)
    {
        oss << "GEOS debug: " << msg;
        if (m_log)
            m_log->get(LogLevel::Debug) << oss.str() << std::endl;
    }
}

GEOSContextHandle_t ErrorHandler::ctx() const
{
    return m_ctx;
}

} // namespace geos
} // namespace pdal

