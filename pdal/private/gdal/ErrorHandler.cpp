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

#include <mutex>

#include "ErrorHandler.hpp"

namespace pdal
{
namespace gdal
{

/**
  Return a reference to the global error handler.

  \return  Reference to the global error handler.
*/
ErrorHandler& ErrorHandler::getGlobalErrorHandler()
{
    static ErrorHandler s_gdalErrorHandler;

    return s_gdalErrorHandler;
}

namespace {

//ABELL - No idea why this is __stdcall
#ifdef _WIN32
void __stdcall trampoline(::CPLErr code, int num, char const* msg)
#else
void trampoline(::CPLErr code, int num, char const* msg)
#endif
{
    ErrorHandler::getGlobalErrorHandler().handle((int)code, num, msg);
}

} // unnamed namespace

/**
  Constructor for a GDAL error handler.
*/
ErrorHandler::ErrorHandler() : m_errorNum(0), m_prevHandler(nullptr)
{
    std::string value;

    // Will return thread-local setting
    const char* set = CPLGetConfigOption("CPL_DEBUG", "");
    m_cplSet = (bool)set;
    m_debug = m_cplSet;
}


/**
  Destructor for a GDAL error handler.
*/
ErrorHandler::~ErrorHandler()
{}


/**
  Set the output destination and debug for the error handler.
  \param log  Pointer to logger.
  \param debug  Whether GDAL debugging should be turned on.
*/
void ErrorHandler::set(LogPtr log, bool debug)
{
    // Set an error handler
    if (m_prevHandler == nullptr)
        m_prevHandler = CPLSetErrorHandler(&trampoline);
    setLog(log);
    setDebug(debug);
}


/**
  Clear the PDAL error handler.
*/
void ErrorHandler::clear()
{
    CPLSetErrorHandler(m_prevHandler);
    m_prevHandler = nullptr;
}


/**
  Set the log destination for GDAL errors.
  \param log  Pointer to the logger.
*/
void ErrorHandler::setLog(LogPtr log)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_log = log;
}

/**
  Set the log state for GDAL logging.
  \param debug  If true, sets the CPL_DEBUG logging option for GDAL
*/
void ErrorHandler::setDebug(bool debug)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_debug = debug;

    if (debug)
        CPLSetThreadLocalConfigOption("CPL_DEBUG", "ON");
    else
        CPLSetThreadLocalConfigOption("CPL_DEBUG", NULL);
}


/**
  Get the number of the last GDAL error.
  \return  Last GDAL error number.
*/
int ErrorHandler::errorNum()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_errorNum;
}

/**
  Callback for GDAL error.
  \param level  Error level
  \param num  Error number
  \param msg  Error message.
*/
void ErrorHandler::handle(int level, int num, char const* msg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ostringstream oss;

    m_errorNum = num;
    if (level == CE_Failure || level == CE_Fatal)
    {
        oss << "GDAL failure (" << num << ") " << msg;
        if (m_log)
            m_log->get(LogLevel::Error) << oss.str() << std::endl;
    }
    else if (m_debug && level == CE_Debug)
    {
        oss << "GDAL debug: " << msg;
        if (m_log)
            m_log->get(LogLevel::Debug) << oss.str() << std::endl;
    }
}

ErrorHandlerSuspender::ErrorHandlerSuspender()
{
    CPLPushErrorHandler(CPLQuietErrorHandler);
}


ErrorHandlerSuspender::~ErrorHandlerSuspender()
{
    (void)CPLPopErrorHandler();
}

} // namespace gdal
} // namespace pdal
