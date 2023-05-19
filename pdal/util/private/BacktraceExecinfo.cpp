/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include "BacktraceImpl.hpp"

#include <execinfo.h> // backtrace
#include <dlfcn.h> // dladdr

namespace pdal
{

Utils::BacktraceEntries Utils::backtraceImpl()
{
    Utils::BacktraceEntries entries;
    const int MAX_STACK_SIZE(100);
    void* buffer[MAX_STACK_SIZE];

    std::size_t size(::backtrace(buffer, MAX_STACK_SIZE));

    for (size_t i = 0; i < size && i < MAX_STACK_SIZE; ++i)
    {
        BacktraceEntry entry;

        entry.addr = buffer[i];

        Dl_info info;
        if (dladdr(entry.addr, &info))
        {
            entry.symname = info.dli_sname;
            entry.libname = info.dli_fname;
            entry.offset = reinterpret_cast<char *>(entry.addr) -
                reinterpret_cast<const char *>(info.dli_saddr);
        }
        entries.push_back(entry);
    }
    return entries;
}

} // namespace pdal
