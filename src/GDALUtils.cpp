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

#include <pdal/GDALUtils.hpp>
#include <pdal/Utils.hpp>
#include <boost/bind/placeholders.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{
namespace gdal
{

Debug::Debug(bool isDebug, pdal::LogPtr log)
: m_isDebug(isDebug)
, m_log(log)
{
    if (m_isDebug)
    {
        const char* gdal_debug = ::pdal::Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            pdal::Utils::putenv("CPL_DEBUG=ON");
        }        
        m_gdal_callback = boost::bind(&Debug::log, this, _1, _2, _3);
    }
    else
    {
        m_gdal_callback = boost::bind(&Debug::error, this, _1, _2, _3);
    }


#if GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 9
    CPLPushErrorHandlerEx(&Debug::trampoline, this);
#else
    CPLPushErrorHandler(&Debug::trampoline);
#endif    
}

void Debug::log(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw pdal::gdal_error(oss.str());
    } else if (code == CE_Debug) {
        oss << "GDAL debug: " << msg;
        m_log->get(logDEBUG) << oss.str() << std::endl;
        return;
    } else {
        return;
    }
}

void Debug::error(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw pdal::gdal_error(oss.str());
    } else {
        return;
    }
}

Debug::~Debug()
{
    CPLPopErrorHandler();

}

}} // namespace pdal::gdal
