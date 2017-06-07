/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#ifndef _WIN32
#include <dlfcn.h>
#endif

#include <pdal/mlang/Environment.hpp>

#include <sstream>
#include <mutex>

namespace pdal
{
namespace mlang
{

static Environment* g_environment=0;

EnvironmentPtr Environment::get()
{
    static std::once_flag flag;

    auto init = []()
    {
        g_environment = new Environment();
    };

    std::call_once(flag, init);

    return g_environment;
}


Environment::Environment()
  : m_engine(0)
{
    m_engine = engOpen("");
    if (!m_engine)
        throw pdal_error("unable to initialize Matlab!");
}


Environment::~Environment()
{
    if (m_engine)
        engClose (m_engine);
}


//
// int Environment::getPythonDataType(Dimension::Type t)
// {
//     using namespace Dimension;
//
//     switch (t)
//     {
//     case Type::Float:
//         return NPY_FLOAT;
//     case Type::Double:
//         return NPY_DOUBLE;
//     case Type::Signed8:
//         return NPY_BYTE;
//     case Type::Signed16:
//         return NPY_SHORT;
//     case Type::Signed32:
//         return NPY_INT;
//     case Type::Signed64:
//         return NPY_LONGLONG;
//     case Type::Unsigned8:
//         return NPY_UBYTE;
//     case Type::Unsigned16:
//         return NPY_USHORT;
//     case Type::Unsigned32:
//         return NPY_UINT;
//     case Type::Unsigned64:
//         return NPY_ULONGLONG;
//     default:
//         return -1;
//     }
//     assert(0);
//
//     return -1;
// }
//


} // namespace mlang
} // namespace pdal

