/******************************************************************************
 * Copyright (c) 2014, Hobu Inc.
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#pragma once

#include <pdal/Metadata.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/pdal_defines.h>
#include <pdal/pdal_export.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/Extractor.hpp>

#ifndef WIN32
#include <sys/fcntl.h>
#include <unistd.h>
#endif

namespace pdal
{
class Options;

namespace Utils
{

inline void printError(const std::string& s)
{
    std::cerr << "PDAL: " << s << std::endl;
    std::cerr << std::endl;
}

inline double toDouble(const Everything& e, Dimension::Type::Enum type)
{
    using namespace Dimension::Type;

    double d = 0;
    switch (type)
    {
    case Unsigned8:
        d = e.u8;
        break;
    case Unsigned16:
        d = e.u16;
        break;
    case Unsigned32:
        d = e.u32;
        break;
    case Unsigned64:
        d = e.u64;
        break;
    case Signed8:
        d = e.s8;
        break;
    case Signed16:
        d = e.s16;
        break;
    case Signed32:
        d = e.s32;
        break;
    case Signed64:
        d = e.s64;
        break;
    case Float:
        d = e.f;
        break;
    case Double:
        d = e.d;
        break;
    default:
        break;
    }
    return d;
}

inline Everything extractDim(Extractor& ext, Dimension::Type::Enum type)
{
    using namespace Dimension::Type;

    Everything e;
    switch (type)
    {
        case Unsigned8:
            ext >> e.u8;
            break;
        case Unsigned16:
            ext >> e.u16;
            break;
        case Unsigned32:
            ext >> e.u32;
            break;
        case Unsigned64:
            ext >> e.u64;
            break;
        case Signed8:
            ext >> e.s8;
            break;
        case Signed16:
            ext >> e.s16;
            break;
        case Signed32:
            ext >> e.s32;
            break;
        case Signed64:
            ext >> e.s64;
            break;
        case Float:
            ext >> e.f;
            break;
        case Double:
            ext >> e.d;
            break;
        case None:
            break;
    }
    return e;
}

inline void insertDim(Inserter& ins, Dimension::Type::Enum type,
    const Everything& e)
{
    using namespace Dimension::Type;

    switch (type)
    {
        case Unsigned8:
            ins << e.u8;
            break;
        case Unsigned16:
            ins << e.u16;
            break;
        case Unsigned32:
            ins << e.u32;
            break;
        case Unsigned64:
            ins << e.u64;
            break;
        case Signed8:
            ins << e.s8;
            break;
        case Signed16:
            ins << e.s16;
            break;
        case Signed32:
            ins << e.s32;
            break;
        case Signed64:
            ins << e.s64;
            break;
        case Float:
            ins << e.f;
            break;
        case Double:
            ins << e.d;
            break;
        case None:
            break;
    }
}



inline MetadataNode toMetadata(const BOX2D& bounds)
{
    MetadataNode output("bbox");
    output.add("minx", bounds.minx);
    output.add("miny", bounds.miny);
    output.add("maxx", bounds.maxx);
    output.add("maxy", bounds.maxy);
    return output;
}

inline MetadataNode toMetadata(const BOX3D& bounds)
{
    MetadataNode output("bbox");
    output.add("minx", bounds.minx);
    output.add("miny", bounds.miny);
    output.add("minz", bounds.minz);
    output.add("maxx", bounds.maxx);
    output.add("maxy", bounds.maxy);
    output.add("maxz", bounds.maxz);
    return output;
}

inline int openProgress(const std::string& filename)
{
#ifdef WIN32
    return -1;
#else
    int fd = open(filename.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd == -1)
    {
        std::string out = "Can't open progress file '";
        out += filename + "'.";
        printError(out);
    }
    return fd;
#endif
}


inline void closeProgress(int fd)
{
#ifdef WIN32
#else
    if (fd >= 0)
        close(fd);
#endif
}


inline void writeProgress(int fd, const std::string& type,
    const std::string& text)
{
#ifdef WIN32
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    if (fd >= 0)
    {
        std::string out = type + ':' + text + '\n';

        // This may error, but we don't care.
        write(fd, out.c_str(), out.length());
    }
#pragma GCC diagnostic pop
#endif
}

std::string PDAL_DLL toJSON(const MetadataNode& m);
void PDAL_DLL toJSON(const MetadataNode& m, std::ostream& o);

} // namespace Utils
} // namespace pdal

