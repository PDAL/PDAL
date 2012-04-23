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

#ifndef PDAL_ENVIRONMENT_H
#define PDAL_ENVIRONMENT_H

#include <pdal/pdal_internal.hpp>

#include <boost/random/mersenne_twister.hpp>

namespace pdal
{

namespace plang
{
class Environment;
}


// this is a singleton: only create it once, and keep it around forever
class PDAL_DLL Environment
{
public:
    static void startup();
    static void shutdown();

    // return the singleton Environment object
    static Environment* get();

#ifdef PDAL_HAVE_PYTHON
    // get the plang (python) environment
    plang::Environment* getPLangEnvironment()
    {
        return m_plangEnvironment;
    }
#endif

    boost::random::mt19937* getRNG()
    {
        return m_rng;
    }

private:
    // ctor and dtor are only called via startup()/shutdown()
    Environment();
    ~Environment();

#ifdef PDAL_HAVE_PYTHON
    plang::Environment* m_plangEnvironment;
#endif

    boost::random::mt19937* m_rng;

    Environment(const Environment&); // nope
    Environment& operator=(const Environment&); // nope
};


} // namespaces

#endif
