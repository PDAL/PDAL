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

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/GDALUtils.hpp>

namespace pdal
{

static GlobalEnvironment* t = 0;
static std::once_flag flag;

GlobalEnvironment& GlobalEnvironment::get()
{
    std::call_once(flag, init);
    return *t;
}


void GlobalEnvironment::startup()
{
    if (t != 0) // sanity check
    {
        throw pdal_error("attempt to reinitialize global environment");
    }
    get();
}


void GlobalEnvironment::shutdown()
{
    if (t == 0) // sanity check
    {
        throw pdal_error("bad global shutdown call -- was called more "
            "than once or was called without corresponding startup");
    }

    delete t;
    t = 0;
}


void GlobalEnvironment::init()
{
    t = new GlobalEnvironment();
}


//
// regular member functions
//

GlobalEnvironment::GlobalEnvironment()
    : m_gdalDebug()
#ifdef PDAL_HAVE_PYTHON
    , m_pythonEnvironment()
#endif
{
}


GlobalEnvironment::~GlobalEnvironment()
{
    if (m_gdalDebug)
    {
        GDALDestroyDriverManager();
    }
}


void GlobalEnvironment::initializeGDAL(LogPtr log)
{
    if (!m_gdalDebug)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_gdalDebug)
        {
            GDALAllRegister();
            OGRRegisterAll();

            m_gdalDebug.reset(new pdal::gdal::GlobalDebug());
        }
    }

    if (m_gdalDebug)
    {
        m_gdalDebug->addLog(log);
    }
    else
    {
        throw pdal_error("Unable to initialize the GDAL environment");
    }
}


gdal::GlobalDebug* GlobalEnvironment::getGDALDebug()
{
    return m_gdalDebug.get();
}


#ifdef PDAL_HAVE_PYTHON
void GlobalEnvironment::createPythonEnvironment()
{
    m_pythonEnvironment.reset(new pdal::plang::PythonEnvironment());
}


plang::PythonEnvironment& GlobalEnvironment::getPythonEnvironment()
{
    if (!m_pythonEnvironment)
        createPythonEnvironment();

    if (m_pythonEnvironment)
        return *m_pythonEnvironment;
    else
        throw pdal_error("Unable to initialize the Python environment!");
}
#endif

} //namespaces
