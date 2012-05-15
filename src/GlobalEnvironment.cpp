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

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/plang/PythonEnvironment.hpp>


namespace pdal
{

GlobalEnvironment* GlobalEnvironment::t = 0;
boost::once_flag GlobalEnvironment::flag = BOOST_ONCE_INIT;


GlobalEnvironment::GlobalEnvironment()
{
    // this should be the not-a-thread thread environment
    (void) createThreadEnvironment(boost::thread::id());

    return;
}


GlobalEnvironment::~GlobalEnvironment()
{
    while (m_threadMap.size())
    {
        thread_map::iterator iter = m_threadMap.begin();
        ThreadEnvironment* env = iter->second;
        delete env;
        m_threadMap.erase(iter);
    }

    return;
}


void GlobalEnvironment::createThreadEnvironment(boost::thread::id id)
{
    ThreadEnvironment* threadEnv = new ThreadEnvironment(id);

    m_threadMap.insert( std::make_pair(id, threadEnv ) );
}


ThreadEnvironment& GlobalEnvironment::getThreadEnvironment(boost::thread::id id)
{
    thread_map::iterator iter =  m_threadMap.find(id);
    if (iter == m_threadMap.end())
        throw pdal_error("bad thread id!");

    ThreadEnvironment* threadEnv = iter->second;

    return *threadEnv;
}


} //namespaces
