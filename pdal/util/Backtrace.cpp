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

#include <pdal/util/Backtrace.hpp>
#include "private/BacktraceImpl.hpp"

#include <vector>
#include <string>

#include <pdal/util/Utils.hpp>

namespace pdal
{

PDAL_EXPORT std::vector<std::string> Utils::backtrace()
{
    std::vector<std::string> lines;
    BacktraceEntries entries = backtraceImpl();

    // Remove the frame for the unwinding itself.
    if (entries.size())
        entries.pop_front();

    size_t maxLibnameLen(0);
    for (auto& be : entries)
    {
        if (be.libname.empty())
            be.libname = "???";
        maxLibnameLen = std::max(maxLibnameLen, be.libname.size());
    }

    // Replace the simple symbol with a better representation if possible.
    for (size_t i = 0; i < entries.size(); ++i)
    {
        BacktraceEntry& be = entries[i];
        std::string line;

        line = std::to_string(i);
        line += std::string(4 - line.size(), ' ');
        // Should the directory info be stripped from the libname?
        line += be.libname;
        line += std::string(maxLibnameLen + 2 - be.libname.size(), ' ');
        if (be.symname.size())
            line += demangle(be.symname);
        else
        {
            std::ostringstream oss;
            intptr_t ip(reinterpret_cast<intptr_t>(be.addr));
            oss << "0x" << std::hex << std::setw(sizeof(ip) * 2) <<
                std::setfill('0') << ip;
            line += oss.str();
        }
        line += " + " + std::to_string(be.offset);
        lines.push_back(line);
    }
    return lines;
}

} // namespace pdal
