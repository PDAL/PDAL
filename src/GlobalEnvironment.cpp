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

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/string_generator.hpp>

#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_HAVE_PYTHON
#include <pdal/plang/PythonEnvironment.hpp>
#endif

#include <pdal/GDALUtils.hpp>


namespace pdal
{

static GlobalEnvironment* t = 0;
static boost::once_flag flag = BOOST_ONCE_INIT;

GlobalEnvironment& GlobalEnvironment::get()
{
    boost::call_once(init, flag);
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
    : m_pythonEnvironment(0)
    , m_bIsGDALInitialized(false)
    , m_gdal_debug(0)
{
    using namespace boost::posix_time;

    ptime epoch(boost::gregorian::date(1970,1,1));
    m_rng.seed((uint32_t)((second_clock::local_time() - epoch).ticks()));
}


void GlobalEnvironment::getGDALEnvironment()
{
    if (!m_bIsGDALInitialized)
    {
        (void) GDALAllRegister();
        m_bIsGDALInitialized = true;
    }
}


GlobalEnvironment::~GlobalEnvironment()
{
#ifdef PDAL_HAVE_PYTHON
    delete m_pythonEnvironment;
    m_pythonEnvironment = 0;
#endif

    if (m_bIsGDALInitialized)
    {
        delete m_gdal_debug;
        (void) GDALDestroyDriverManager();
        m_bIsGDALInitialized = false;
    }
}


#ifdef PDAL_HAVE_PYTHON
void GlobalEnvironment::createPythonEnvironment()
{
    m_pythonEnvironment = new pdal::plang::PythonEnvironment();
}
#endif


plang::PythonEnvironment& GlobalEnvironment::getPythonEnvironment()
{
#ifdef PDAL_HAVE_PYTHON
    if (!m_pythonEnvironment)
        (void) createPythonEnvironment();
#endif

    if (m_pythonEnvironment)
        return *m_pythonEnvironment;
    else
        throw pdal_error("Unable to initialize the Python environment!");
}

pdal::gdal::GlobalDebug* GlobalEnvironment::getGDALDebug()
{
    getGDALEnvironment();
    if (m_gdal_debug == 0)
        m_gdal_debug = new pdal::gdal::GlobalDebug();
    return m_gdal_debug;
}

boost::uuids::uuid GlobalEnvironment::generateUUID()
{
    boost::uuids::basic_random_generator<boost::mt19937> gen(&m_rng);
    return gen();
}


boost::uuids::uuid GlobalEnvironment::generateUUID(const std::string& s)
{
    boost::uuids::string_generator gen;
    return gen(s);
}

} //namespaces
