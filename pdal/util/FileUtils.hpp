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

#include <cassert>
#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "pdal_util_export.hpp"

namespace pdal
{

namespace FileUtils
{
    /**
      Open an existing file for reading.

      \param filename  Filename.
      \param asBinary  Read as binary file (don't convert /r/n to /n)
      \return  Pointer to opened stream.
    */
    PDAL_DLL std::istream* openFile(std::string const& filename,
        bool asBinary=true);

    /**
      Create/truncate a file and open for writing.

      \param filename  Filename.
      \param asBinary  Write as binary file (don't convert /n to /r/n)
      \return  Point to opened stream.
    */
    PDAL_DLL std::ostream* createFile(std::string const& filename,
        bool asBinary=true);

    /**
      Open an existing file for write

      \param filename  Filename.
      \param asBinary  Write as binary file (don't convert /n to /r/n)
      \return  Point to opened stream.
    */
    PDAL_DLL std::ostream* openExisting(std::string const& filename,
        bool asBinary=true);


    /**
      Determine if a directory exists.

      \param dirname  Name of directory.
      \return  Whether a directory exists.
    */
    PDAL_DLL bool directoryExists(const std::string& dirname);

    /**
      Create a directory.

      \param dirname  Directory name.
      \return  Whether the directory was created.
    */
    PDAL_DLL bool createDirectory(const std::string& dirname);

    /**
      Create all directories in the provided path.

      \param dirname  Path name.
      \return  \false on failure
    */
    PDAL_DLL bool createDirectories(const std::string& path);

    /**
      Delete a directory and its contents.

      \param dirname  Directory name.
    */
    PDAL_DLL void deleteDirectory(const std::string& dirname);

    /**
      List the contents of a directory.

      \param dirname  Name of directory to list.
      \return  List of entries in the directory.
    */
    PDAL_DLL std::vector<std::string> directoryList(const std::string& dirname);

    /**
      Close a file created with createFile.

      \param ofs  Pointer to stream to close.
    */
    PDAL_DLL void closeFile(std::ostream* ofs);

    /**
      Close a file created with openFile.

      \param ifs  Pointer to stream to close.
    */
    PDAL_DLL void closeFile(std::istream* ifs);

    /**
      Delete a file.

      \param filename  Name of file to delete.
      \return  \c true if successful, \c false otherwise
    */
    PDAL_DLL bool deleteFile(const std::string& filename);

    /**
      Rename a file.

      \param dest  Desired filename.
      \param src   Source filename.
    */
    PDAL_DLL void renameFile(const std::string& dest, const std::string& src);

    /**
      Determine if a file exists.

      \param  Filename.
      \return  Whether the file exists.
    */
    PDAL_DLL bool fileExists(const std::string& filename);

    /**
      Get the size of a file.

      \param filename  Filename.
      \return  Size of file.
    */
    PDAL_DLL uintmax_t fileSize(const std::string& filename);

    /**
      Read a file into a string.

      \param filename  Filename.
      \return  File contents as a string
    */
    PDAL_DLL std::string readFileIntoString(const std::string& filename);

    /**
      Get the current working directory with trailing separator.

      \return  The current working directory.
    */
    PDAL_DLL std::string getcwd();

    /**
      Return the file component of the given path,
      e.g. "d:/foo/bar/a.c" -> "a.c"

      \param path  Path from which to extract file component.
      \return  File part of path.
    */
    PDAL_DLL std::string getFilename(const std::string& path);

    /**
      Return the directory component of the given path,
      e.g. "d:/foo/bar/a.c" -> "d:/foo/bar/"

      \param path  Path from which to extract directory component.
      \return  Directory part of path.
    */
    PDAL_DLL std::string getDirectory(const std::string& path);

    /**
      Determine if the path is an absolute path.

      \param path  Path to test.
      \return  Whether the path is absolute.
    */
    PDAL_DLL bool isAbsolutePath(const std::string& path);

    /**
      Determine if path is a directory.

      \param path  Directory to check.
      \return  Whether the path represents a directory.
    */
    PDAL_DLL bool isDirectory(const std::string& path);

    /**
      Return the path with all ".", ".." and symbolic links removed.
      The file must exist.

      \param filename  Name of file to convert to canonical path.
      \return  Canonical version of provided filename, or empty string.
    */
    PDAL_DLL std::string toCanonicalPath(std::string filename);


    /**
      If the filename is an absolute path, just return it otherwise,
      make it absolute (relative to current working dir) and return it.

      \param filename  Name of file to convert to absolute path.
      \return  Absolute version of provided filename.
    */
    PDAL_DLL std::string toAbsolutePath(const std::string& filename);

    /**
      If the filename is an absolute path, just return it otherwise,
      make it absolute (relative to base dir) and return that.

      \param filename  Name of file to convert to absolute path.
      \param base  Base name to use.
      \return  Absolute version of provided filename relative to base.
    */
    PDAL_DLL std::string toAbsolutePath(const std::string& filename,
        const std::string base);
    
    /**
      Get the file creation and modification times.

      \param filename  Filename.
      \param createTime  Pointer to creation time structure.
      \param modTime  Pointer to modification time structure.
    */
    PDAL_DLL void fileTimes(const std::string& filename, struct tm *createTime,
        struct tm *modTime);

    /**
      Return the extension of the filename, including the separator (.).

      \param path  File path from which to extract extension.
      \return  Extension of filename.
    */
    PDAL_DLL std::string extension(const std::string& path);

    /**
      Return the filename stripped of the extension.  . and .. are returned
      unchanged.

      \param path  File path from which to extract file stem.
      \return  Stem of filename.
    */
    PDAL_DLL std::string stem(const std::string& path);

    /**
      Expand a filespec to a list of files.

      \param filespec  File specification to expand.
      \return  List of files that correspond to provided file specification.
    */
    PDAL_DLL std::vector<std::string> glob(std::string filespec);


    struct MapContext
    {
    public:
        MapContext() : m_fd(-1), m_addr(nullptr)
        {}

        PDAL_DLL void *addr() const
        { return m_addr; }
        PDAL_DLL std::string what() const
        { return m_error; }

        int m_fd;
        uintmax_t m_size;
        void *m_addr;
        std::string m_error;
#ifdef _WIN32
        HANDLE m_handle;
#endif
    };
    /**
      Map a file to memory.
      \param filename  Filename to map.
      \param readOnly  Must be true at this time.
      \param pos       Starting position of file to map.
      \param size      Number of bytes in file to map.
      \return  MapContext.  addr() gets the mapped address.  what() gets
         any error message.  addr() returns nullptr on error.
    */
    PDAL_DLL MapContext mapFile(const std::string& filename, bool readOnly = true,
        uintmax_t pos = 0, uintmax_t size = 0);

    /**
      Unmap a previously mapped file.
      \param ctx  Previously returned MapContext
      \return  MapContext indicating current state of the file mapping.
    */
    PDAL_DLL MapContext unmapFile(MapContext ctx);

} // namespace FileUtils
} // namespace pdal
