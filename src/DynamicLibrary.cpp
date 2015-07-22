/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

// The DynamicLibrary was modeled very closely after the work of Gigi Sayfan in
// the Dr. Dobbs article:
// http://www.drdobbs.com/cpp/building-your-own-plugin-framework-part/206503957
// The original work was released under the Apache License v2.

#ifdef _WIN32
  #include <Windows.h>
#else
  #include <dlfcn.h>
#endif

#include "DynamicLibrary.h"
#include <sstream>
#include <iostream>

namespace pdal
{

DynamicLibrary::~DynamicLibrary()
{
    if (m_handle)
    {
#ifndef _WIN32
        ::dlclose(m_handle);
#else
        ::FreeLibrary((HMODULE)m_handle);
#endif
    }
}

DynamicLibrary *DynamicLibrary::load(const std::string &name, 
    std::string &errorString)
{
    if (name.empty()) 
    {
        errorString = "Empty path.";
        return NULL;
    }

    void *handle = NULL;

#ifdef _WIN32
    handle = ::LoadLibraryA(name.c_str());
    if (handle == NULL)
    {
        DWORD errorCode = ::GetLastError();
        std::stringstream ss;
        ss << std::string("LoadLibrary(") << name 
            << std::string(") Failed. errorCode: ") 
            << errorCode; 
        errorString = ss.str();
    }
#else
    handle = ::dlopen(name.c_str(), RTLD_NOW);
    if (!handle) 
    {
        std::string dlErrorString;
        const char *zErrorString = ::dlerror();
        if (zErrorString)
            dlErrorString = zErrorString;
        errorString += "Failed to load \"" + name + '"';
        if (dlErrorString.size())
            errorString += ": " + dlErrorString;
        return NULL;
    }
#endif
    return new DynamicLibrary(handle);
}


void *DynamicLibrary::getSymbol(const std::string& symbol)
{
    if (!m_handle)
        return NULL;

    void *sym;
#ifdef _WIN32
    sym = ::GetProcAddress((HMODULE)m_handle, symbol.c_str());
#else
    sym = ::dlsym(m_handle, symbol.c_str());
#endif
    return sym;
}

} // namespace pdal

