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

#include <pdal/pdal_types.hpp>
#include <pdal/Log.hpp>

#include <cstdarg>
#include <functional>

#include <geos_c.h>


namespace pdal
{

namespace geos
{

class PDAL_DLL ErrorHandler
{
public:

    ErrorHandler(bool isDebug, pdal::LogPtr log);
    ~ErrorHandler();
    ErrorHandler(const ErrorHandler& other );
    void setup(bool isDebug, pdal::LogPtr log);


#ifdef GEOSGContext_setErrorMessageHandler_r
    static void GEOS_DLL error_trampoline(const char* message, void* userdata)
    {
        ErrorHandler* debug =
            static_cast<ErrorHandler*>(userdata);
        if (!debug)
            return;
        debug->m_geos_callback(message);
        if (!debug->m_log->get()) return;
    }
#else

    static void GEOS_DLL error_trampoline(const char* message, ...)
    {
    va_list args;

    va_start(args, message);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), message, args);
    std::cerr<< "GEOS error: " << buf << std::endl;

    va_end(args);

    }
#endif

#ifdef GEOSContext_setNoticeHandler_r
    static void GEOS_DLL notice_trampoline(const char* message, void* userdata)
    {
        ErrorHandler* debug =
            static_cast<ErrorHandler*>(userdata);
        if (!debug)
            return;
        debug->m_geos_callback(message);
        if (!debug->m_log->get()) return;
    }
#else

    static void GEOS_DLL notice_trampoline(const char* message, ...)
    {
    va_list args;

    va_start(args, message);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), message, args);
    std::cerr<< "GEOS notice tramp: " << buf << std::endl;

    va_end(args);

    }
#endif

    void log(char const* msg);
    void error(char const* msg);

    GEOSContextHandle_t ctx;

    inline LogPtr getLogger() const { return m_log; }
    inline void setLogger(LogPtr logger) { m_log = logger; }

private:
    std::function<void(const char*)> m_geos_callback;
    bool m_isDebug;
    pdal::LogPtr m_log;
};






} // end geos
} // namespace pdal

