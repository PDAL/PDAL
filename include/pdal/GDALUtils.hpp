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

#include <pdal/pdal_internal.hpp>

#include <pdal/Log.hpp>

#include <sstream>
#include <vector>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <cpl_port.h>
#include "gdal.h"
#include <cpl_vsi.h>
#include <ogr_api.h>

namespace pdal
{
namespace gdal
{


class PDAL_DLL ErrorHandler 
{
public:

    ErrorHandler(bool isDebug, pdal::LogPtr log);
    ~ErrorHandler();

    static void CPL_STDCALL trampoline(::CPLErr code, int num, char const* msg)
    {
        ErrorHandler* debug = static_cast<ErrorHandler*>(CPLGetErrorHandlerUserData());
        if (!debug)
            return;

        // if (!debug->m_log->get()) return;
        debug->m_gdal_callback(code, num, msg);
    }

    void log(::CPLErr code, int num, char const* msg);
    void error(::CPLErr code, int num, char const* msg);

    inline LogPtr getLogger() const { return m_log; }
    inline void setLogger(LogPtr logger) { m_log = logger; }

private:
    boost::function<void(CPLErr, int, char const*)> m_gdal_callback;
    bool m_isDebug;
    pdal::LogPtr m_log;
};

class PDAL_DLL VSILFileBuffer
{
public:
    typedef boost::iostreams::seekable_device_tag category;
    typedef char char_type;

    VSILFileBuffer(VSILFILE* fp);

    std::streamsize read(char* s, std::streamsize n);
    std::streamsize write(const char* s, std::streamsize n);
    std::streampos seek(boost::iostreams::stream_offset off, std::ios_base::seekdir way);

private:
    VSILFILE* m_fp;
};


}
} // namespace pdal::gdal

#endif
