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

#pragma once

#include <pdal/pdal_internal.hpp>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>

#include "pdal_util_export.hpp"

namespace pdal
{

namespace FileUtils
{
    // open existing file for reading
    PDAL_DLL std::istream* openFile(std::string const& filename,
        bool asBinary=true);

    // open new file for writing
    PDAL_DLL std::ostream* createFile(std::string const& filename,
        bool asBinary=true);

    PDAL_DLL bool directoryExists(const std::string& dirname);
    PDAL_DLL bool createDirectory(const std::string& dirname);
    PDAL_DLL void deleteDirectory(const std::string& dirname);
    PDAL_DLL StringList directoryList(const std::string& dirname);

    PDAL_DLL void closeFile(std::ostream* ofs);
    PDAL_DLL void closeFile(std::istream* ifs);

    PDAL_DLL bool deleteFile(const std::string& filename);
    PDAL_DLL void renameFile(const std::string& dest, const std::string& src);
    PDAL_DLL bool fileExists(const std::string& filename);
    PDAL_DLL uintmax_t fileSize(const std::string& filename);

    // reads a file into a text string for you
    PDAL_DLL std::string readFileIntoString(const std::string& filename);

    // return current working dir
    // the result will always have a trailing '/'
    PDAL_DLL std::string getcwd();

    // return the file component of the given path,
    // e.g. "d:/foo/bar/a.c" -> "a.c"
    PDAL_DLL std::string getFilename(const std::string& path);

    // return the directory component of the given path,
    // e.g. "d:/foo/bar/a.c" -> "d:/foo/bar"
    // the result will always have a trailing '/'
    PDAL_DLL std::string getDirectory(const std::string& path);

    // returns true iff the path is not relative
    PDAL_DLL bool isAbsolutePath(const std::string& path);

    // Returns true if path is a directory.
    PDAL_DLL bool isDirectory(const std::string& path);

    // if the filename is an absolute path, just return it
    // otherwise, make it absolute (relative to current working dir)
    // and return that
    PDAL_DLL std::string toAbsolutePath(const std::string& filename);

    // if the filename is an absolute path, just return it
    // otherwise, make it absolute (relative to base dir) and return that
    //
    // note: if base dir is not absolute, first make it absolute via
    // toAbsolutePath(base)
    PDAL_DLL std::string toAbsolutePath(const std::string& filename,
        const std::string base);
    
    PDAL_DLL void fileTimes(const std::string& filename, struct tm *createTime,
        struct tm *modTime);

    /// Return the extension of the filename, including the separator (.).
    PDAL_DLL std::string extension(const std::string& path);

    /// Return the filename stripped of the extension.  . and .. are returned
    /// unchanged.
    PDAL_DLL std::string stem(const std::string& path);
}

} // namespace pdal
