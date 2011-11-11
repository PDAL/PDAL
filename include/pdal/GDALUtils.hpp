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

#ifndef INCLUDED_GDALUTILS_HPP
#define INCLUDED_GDALUTILS_HPP

#include <pdal/pdal.hpp>
#include <pdal/pdal_error.hpp>

#include <pdal/Log.hpp>

#include <sstream>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#ifdef PDAL_HAVE_GDAL
#include <cpl_port.h>
#include "gdal.h"
// #include "gdal_priv.h"
#endif

namespace pdal
{
namespace gdal {


class PDAL_DLL Debug
{
public:

    Debug(bool isDebug, pdal::LogPtr log);
    ~Debug() {}

    static void CPL_STDCALL trampoline(::CPLErr code, int num, char const* msg)
    {
#if GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 9
        static_cast<Debug*>(CPLGetErrorHandlerUserData())->m_gdal_callback(code, num, msg);
#else
        if (code == CE_Failure || code == CE_Fatal) {
            std::ostringstream oss;
            oss <<"GDAL Failure number=" << num << ": " << msg;
            throw gdal_error(oss.str());
        } else if (code == CE_Debug) {
            std::clog << " (no log control stdlog) GDAL debug: " << msg << std::endl;
        } else {
            return;
        }
#endif
    }
    
    void CPL_STDCALL log(::CPLErr code, int num, char const* msg);
    void CPL_STDCALL error(::CPLErr code, int num, char const* msg);


private:
    boost::function<void(CPLErr, int, char const*)> m_gdal_callback;
    bool m_isDebug;
    pdal::LogPtr m_log;
};

}} // namespace pdal::gdal

#endif
