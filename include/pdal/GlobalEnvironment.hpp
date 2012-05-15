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

#ifndef PDAL_GLOBAL_ENVIRONMENT_H
#define PDAL_GLOBAL_ENVIRONMENT_H

#include <pdal/pdal_internal.hpp>
#include <pdal/ThreadEnvironment.hpp>
#include <boost/thread/once.hpp>
#include <map>

namespace pdal
{

//namespace plang
//{
//class PythonEnvironment;
//}


class PDAL_DLL GlobalEnvironment
{
public:
    static GlobalEnvironment& get()
    {
        boost::call_once(init, flag);
        return *t;
    }

    static void startup()
    {
        get();
    }
    static void shutdown()
    {
        delete t;
        t = 0;
    }

private:
    static void init() // never throws
    {
        t = new GlobalEnvironment();
    }

    GlobalEnvironment();
    ~GlobalEnvironment();

public:
    // create a new thread context
    void createThreadEnvironment(boost::thread::id);

    // returns the thread context for a thread
    // with no args, returns the not-a-thread thread environment
    ThreadEnvironment& getThreadEnvironment(boost::thread::id=boost::thread::id());

    //
    // forwarded functions
    //
#ifdef PDAL_HAVE_PYTHON
    // get the plang (python) environment
    plang::PythonEnvironment& getPythonEnvironment()
    {
        return getThreadEnvironment().getPythonEnvironment();
    }
#endif

    boost::random::mt19937* getRNG()
    {
        return getThreadEnvironment().getRNG();
    }

private:
    static GlobalEnvironment* t;
    static boost::once_flag flag;

    typedef std::map<boost::thread::id, ThreadEnvironment*> thread_map;
    thread_map m_threadMap;

    GlobalEnvironment(const GlobalEnvironment&); // nope
    GlobalEnvironment& operator=(const GlobalEnvironment&); // nope
};

typedef GlobalEnvironment TheGlobalEnvironment;

} // namespaces

#endif
