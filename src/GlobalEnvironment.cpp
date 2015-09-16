/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

//#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/GDALUtils.hpp>

namespace pdal
{

static GlobalEnvironment* s_environment = 0;

GlobalEnvironment& GlobalEnvironment::get()
{
    static std::once_flag flag;

    auto init = []()
    {
        s_environment = new GlobalEnvironment();
    };

    std::call_once(flag, init);
    return *s_environment;
}


void GlobalEnvironment::startup()
{
    if (s_environment)
        throw pdal_error("attempt to reinitialize global environment");
    get();
}


void GlobalEnvironment::shutdown()
{
    if (!s_environment)
        throw pdal_error("bad global shutdown call -- was called more "
            "than once or was called without corresponding startup");
    delete s_environment;
    s_environment = 0;
}


GlobalEnvironment::GlobalEnvironment() : m_gdalDebug()
{}


GlobalEnvironment::~GlobalEnvironment()
{
    if (m_gdalDebug)
        GDALDestroyDriverManager();
}


void GlobalEnvironment::initializeGDAL(LogPtr log, bool gdalDebugOutput)
{
    static std::once_flag flag;

    auto init = [this](LogPtr log, bool gdalDebugOutput) -> void
    {
        GDALAllRegister();
        OGRRegisterAll();
        m_gdalDebug.reset(new gdal::ErrorHandler(gdalDebugOutput, log));
    };

    std::call_once(flag, init, log, gdalDebugOutput);
}


} //namespaces

