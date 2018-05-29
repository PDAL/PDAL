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

#include "BacktraceImpl.hpp"

#include <execinfo.h> // backtrace
#include <dlfcn.h> // dladdr

namespace pdal
{

Utils::BacktraceEntries Utils::backtraceImpl()
{
    std::vector<std::string> lines;
    const int MAX_STACK_SIZE(100);
    void* buffer[MAX_STACK_SIZE];
    std::vector<std::string> prefixes;
    size_t maxPrefix(0);

    std::size_t size(::backtrace(buffer, MAX_STACK_SIZE));
    char** symbols(backtrace_symbols(buffer, size));

    // Store strings and free symbols.  Start at 1 to remove this function
    // from the stack.
    for (std::size_t i(1); i < size; ++i)
    {
        lines.push_back(symbols[i]);
        const std::string& symbol = lines.back();
        std::string prefix;
        std::size_t pos = symbol.find("0x");
        if (pos != std::string::npos)
            prefix = symbol.substr(0, pos);
        else
            prefix = std::to_string(i) + "  ???";
        trimTrailing(prefix);
        prefixes.push_back(prefix);
        maxPrefix = (std::max)(prefix.size(), maxPrefix);
    }
    free(symbols);

    // Replace the simple symbol with a better representation if possible.
    for (std::size_t i(1); i < size; ++i)
    {
        std::string& symbol = lines[i - 1];
        std::string& prefix = prefixes[i - 1];
        prefix = prefix + std::string(maxPrefix + 2 - prefix.size(), ' ');

        Dl_info info;
        if (dladdr(buffer[i], &info))
        {
            const std::size_t offset(static_cast<char*>(buffer[i]) -
                        static_cast<char*>(info.dli_saddr));

            // Replace the address and mangled name with a human-readable
            // name.
            symbol = prefix + demangle(info.dli_sname) + " + " +
                std::to_string(offset);
        }
        else
        {
            symbol = symbol.substr(maxPrefix + 2);
            trimLeading(symbol);
            symbol = prefix + symbol;
        }
    }
    return lines;
}

} // namespace pdal
