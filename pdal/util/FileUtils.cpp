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

#include <sys/stat.h>

#include <iostream>
#include <sstream>
#ifndef WIN32
#include <glob.h>
#else
#include <Windows.h>
#endif

#include <boost/filesystem.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_types.hpp>

namespace pdal
{

namespace
{

bool isStdin(std::string filename)
{
    return Utils::toupper(filename) == "STDIN";
}

bool isStdout(std::string filename)
{
    return Utils::toupper(filename) == "STOUT" ||
        Utils::toupper(filename) == "STDOUT";
}

std::string addTrailingSlash(std::string path)
{
    if (path[path.size() - 1] != '/' && path[path.size() - 1] != '\\')
        path += "/";
    return path;
}

} // unnamed namespace

namespace FileUtils
{

std::istream *openFile(std::string const& filename, bool asBinary)
{
    std::ifstream *ifs = nullptr;

    std::string name(filename);
    if (isStdin(name))
        return &std::cin;

    if (!FileUtils::fileExists(name))
        return nullptr;

    std::ios::openmode mode = std::ios::in;
    if (asBinary)
        mode |= std::ios::binary;

    ifs = new std::ifstream(name, mode);
    if (!ifs->good())
    {
        delete ifs;
        return nullptr;
    }
    return ifs;
}


std::ostream *createFile(std::string const& name, bool asBinary)
{
    if (isStdout(name))
        return &std::cout;

    std::ios::openmode mode = std::ios::out;
    if (asBinary)
        mode |= std::ios::binary;

    std::ostream *ofs = new std::ofstream(name, mode);
    if (!ofs->good())
    {
        delete ofs;
        return nullptr;
    }
    return ofs;
}


bool directoryExists(const std::string& dirname)
{
    //ABELL - Seems we should be calling is_directory
    return pdalboost::filesystem::exists(dirname);
}


bool createDirectory(const std::string& dirname)
{
    return pdalboost::filesystem::create_directory(dirname);
}


void deleteDirectory(const std::string& dirname)
{
    pdalboost::filesystem::remove_all(dirname);
}


std::vector<std::string> directoryList(const std::string& dir)
{
    std::vector<std::string> files;

    try
    {
        pdalboost::filesystem::directory_iterator it(dir);
        pdalboost::filesystem::directory_iterator end;
        while (it != end)
        {
            files.push_back(it->path().string());
            it++;
        }
    }
    catch (pdalboost::filesystem::filesystem_error&)
    {
        files.clear();
    }
    return files;
}


void closeFile(std::ostream *out)
{
    // An ofstream is closeable and deletable, but
    // an ostream like &cout isn't.
    if (!out)
        return;
    std::ofstream *ofs = dynamic_cast<std::ofstream *>(out);
    if (ofs)
    {
        ofs->close();
        delete ofs;
    }
}


void closeFile(std::istream* in)
{
    // An ifstream is closeable and deletable, but
    // an istream like &cin isn't.
    if (!in)
        return;
    std::ifstream *ifs = dynamic_cast<std::ifstream *>(in);
    if (ifs)
    {
        ifs->close();
        delete ifs;
    }
}


bool deleteFile(const std::string& file)
{
    return pdalboost::filesystem::remove(file);
}


void renameFile(const std::string& dest, const std::string& src)
{
    pdalboost::filesystem::rename(src, dest);
}


bool fileExists(const std::string& name)
{
    if (isStdin(name))
        return true;

    try
    {
        return pdalboost::filesystem::exists(name);
    }
    catch (pdalboost::filesystem::filesystem_error&)
    {
    }
    return false;
}


uintmax_t fileSize(const std::string& file)
{
    return pdalboost::filesystem::file_size(file);
}


std::string readFileIntoString(const std::string& filename)
{
    std::string str;

    std::istream* stream = openFile(filename, false);
    if (stream)
    {
        str.assign((std::istreambuf_iterator<char>(*stream)),
            std::istreambuf_iterator<char>());
        closeFile(stream);
    }
    return str;
}


std::string getcwd()
{
    const pdalboost::filesystem::path p = pdalboost::filesystem::current_path();
    return addTrailingSlash(p.string());
}


/***
// Non-boost alternative.  Requires file existence.
std::string toAbsolutePath(const std::string& filename)
{
    std::string result;

#ifdef WIN32
    char buf[MAX_PATH]
    if (GetFullPathName(filename.c_str(), MAX_PATH, buf, NULL))
        result = buf;
#else
    char buf[PATH_MAX];
    if (realpath(filename.c_str(), buf))
        result = buf;
#endif
    return result;
}
***/

// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to current working dir) and return that
std::string toAbsolutePath(const std::string& filename)
{
    return pdalboost::filesystem::absolute(filename).string();
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to base dir) and return that
//
// note: if base dir is not absolute, first make it absolute via
// toAbsolutePath(base)
std::string toAbsolutePath(const std::string& filename, const std::string base)
{
    const std::string newbase = toAbsolutePath(base);
    return pdalboost::filesystem::absolute(filename, newbase).string();
}

std::string getFilename(const std::string& path)
{
#ifdef _WIN32
    std::string pathsep("\\/");
#else
    char pathsep = '/';
#endif

    std::string::size_type pos = path.find_last_of(pathsep);
    if (pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
}


// Get the directory part of a filename.
std::string getDirectory(const std::string& path)
{
    const pdalboost::filesystem::path dir =
         pdalboost::filesystem::path(path).parent_path();
    return addTrailingSlash(dir.string());
}


std::string stem(const std::string& path)
{
    std::string f = getFilename(path);
    if (f != "." && f != "..")
    {
        std::string::size_type pos = f.find_last_of(".");
        if (pos != std::string::npos)
            f = f.substr(0, pos);
    }
    return f;
}


// Determine if the path represents a directory.
bool isDirectory(const std::string& path)
{
    return pdalboost::filesystem::is_directory(path);
}

// Determine if the path is an absolute path
bool isAbsolutePath(const std::string& path)
{
    return pdalboost::filesystem::path(path).is_absolute();
}


void fileTimes(const std::string& filename, struct tm *createTime,
    struct tm *modTime)
{
#ifdef WIN32
    struct _stat statbuf;
    _stat(filename.c_str(), &statbuf);

    if (createTime)
        *createTime = *gmtime(&statbuf.st_ctime);
    if (modTime)
        *modTime = *gmtime(&statbuf.st_mtime);
#else
    struct stat statbuf;
    stat(filename.c_str(), &statbuf);

    if (createTime)
        gmtime_r(&statbuf.st_ctime, createTime);
    if (modTime)
        gmtime_r(&statbuf.st_mtime, modTime);
#endif
}


std::string extension(const std::string& filename)
{
    auto idx = filename.find_last_of('.');
    if (idx == std::string::npos)
        return std::string();
    return filename.substr(idx);
}


std::vector<std::string> glob(std::string path)
{
    std::vector<std::string> filenames;
#ifdef WIN32
    WIN32_FIND_DATA ffd;
    HANDLE handle = FindFirstFile(path.c_str(), &ffd);

    if (INVALID_HANDLE_VALUE == handle)
        return filenames;

    size_t found = path.find_last_of("/\\");
    do
    {
        if (found == std::string::npos)
            filenames.push_back(ffd.cFileName);
        else
            filenames.push_back(path.substr(0, found) + "\\" + ffd.cFileName);

    } while (FindNextFile(handle, &ffd) != 0);
    FindClose(handle);
#else
    glob_t glob_result;

    ::glob(path.c_str(), GLOB_NOSORT, NULL, &glob_result);
    for (unsigned int i = 0; i < glob_result.gl_pathc; ++i)
    {
        std::string filename = glob_result.gl_pathv[i];
        filenames.push_back(filename);
    }
    globfree(&glob_result);
#endif
    return filenames;
}

} // namespace FileUtils

} // namespace pdal

