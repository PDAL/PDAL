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

#include <fcntl.h>
#include <sys/stat.h>

#include <iostream>
#include <sstream>
#ifndef _WIN32
#include <glob.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <io.h>
#include <codecvt>
#endif

#include <filesystem>
namespace fs = std::filesystem;



#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_types.hpp>

#include "pdal_util_internal.hpp"

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
#ifdef PDAL_WIN32_STL
std::wstring toNative(const std::string& in)
{
    if (in.empty())
        return std::wstring();

    // The first call determines the length of the conversion. The second does the
    // actual conversion.
    int len = MultiByteToWideChar(CP_UTF8, 0, in.data(), in.length(), nullptr, 0);
    std::wstring out(len, 0);
    if (MultiByteToWideChar(CP_UTF8, 0, in.data(), in.length(), out.data(), len) == 0)
    {
        char buf[200] {};
        len = FormatMessageA(0, 0, GetLastError(), 0, buf, 199, 0);
        throw pdal_error("Can't convert UTF8 to UTF16: " + std::string(buf, len));
    }
    return out;
}

std::string fromNative(const std::wstring& in)
{
    if (in.empty())
        return std::string();

    // The first call determines the length of the conversion. The second does the
    // actual conversion.
    int len = WideCharToMultiByte(CP_UTF8, 0, in.data(), in.length(), nullptr, 0, nullptr, nullptr);
    std::string out(len, 0);
    if (WideCharToMultiByte(CP_UTF8, 0, in.data(), in.length(), out.data(), len,
        nullptr, nullptr) == 0)
    {
        int err = GetLastError();
        char buf[200] {};
        len = FormatMessageA(0, 0, GetLastError(), 0, buf, 199, 0);
        throw pdal_error("Can't convert UTF16 to UTF8: " + std::string(buf, len));
    }
    return out;
}
#else // Unix, OSX, MinGW
std::string toNative(const std::string& in)
{
    return in;
}

std::string fromNative(const std::string& in)
{
    return in;
}
#endif


std::istream *openFile(std::string const& filename, bool asBinary)
{
    if (filename[0] == '~')
        throw pdal::pdal_error("PDAL does not support shell expansion");

    std::ifstream *ifs = nullptr;

    std::string name(filename);
    if (isStdin(name))
        return &std::cin;

    if (!FileUtils::fileExists(name))
        return nullptr;

    std::ios::openmode mode = std::ios::in;
    if (asBinary)
        mode |= std::ios::binary;

    ifs = new Utils::ClassicLocaleStream<std::ifstream>(toNative(name), mode);
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

    std::ostream *ofs = new Utils::ClassicLocaleStream<std::ofstream>(toNative(name), mode);
    if (!ofs->good())
    {
        delete ofs;
        return nullptr;
    }
    return ofs;
}


std::ostream *openExisting(const std::string& name, bool asBinary)
{
    std::ios::openmode mode = std::ios::out | std::ios::in;
    if (asBinary)
        mode |= std::ios::binary;

    std::ostream *ofs = new Utils::ClassicLocaleStream<std::ofstream>(toNative(name), mode);
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
    return fs::exists(toNative(dirname));
}


bool createDirectory(const std::string& dirname)
{
    return fs::create_directory(toNative(dirname));
}


bool createDirectories(const std::string& dirname)
{
    // Need to strip any /'s off the end because windows and unix
    // create_directories seem to behave differently
    std::string s(dirname);
    if('/' == s.back())
        s.pop_back();

    return fs::create_directories(toNative(s));
}


void deleteDirectory(const std::string& dirname)
{
    fs::remove_all(toNative(dirname));
}


std::vector<std::string> directoryList(const std::string& dir)
{
    std::vector<std::string> files;

    try
    {
        fs::directory_iterator it(toNative(dir));
        fs::directory_iterator end;
        while (it != end)
        {
            files.push_back(it->path().u8string());
            it++;
        }
    }
    catch (fs::filesystem_error& )
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
    return fs::remove(toNative(file));
}


void renameFile(const std::string& dest, const std::string& src)
{
    fs::rename(toNative(src), toNative(dest));
}


bool fileExists(const std::string& name)
{
    if (isStdin(name))
        return true;

    try
    {
        return fs::exists(toNative(name));
    }
    catch (fs::filesystem_error&)
    {
    }
    return false;
}


/// \return  0 on error or invalid file type.
uintmax_t fileSize(const std::string& file)
{
    std::error_code ec;
    uintmax_t size = fs::file_size(toNative(file), ec);
    if (ec)
        size = 0;
    return size;
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
    const fs::path p = fs::current_path();
    return addTrailingSlash(p.u8string());
}


std::string toCanonicalPath(std::string filename)
{
    return fs::weakly_canonical(toNative(filename)).u8string();
}

// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to current working dir) and return that
std::string toAbsolutePath(const std::string& filename)
{
    return fs::absolute(toNative(filename)).u8string();
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to base dir) and return that
//
// note: if base dir is not absolute, first make it absolute via
// toAbsolutePath(base)
std::string toAbsolutePath(const std::string& filename, const std::string base)
{
    const std::string newbase = toAbsolutePath(base);
    fs::path f (toNative(filename));
    fs::path b (toNative(newbase));

    fs::path fb = b / f ;
    return fb.string();
}


std::string getFilename(const std::string& path)
{
#ifdef _WIN32
    std::string pathsep("\\/");
#else
    char pathsep = Utils::dirSeparator;
#endif

    std::string::size_type pos = path.find_last_of(pathsep);
    if (pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
}


// Get the directory part of a filename.
std::string getDirectory(const std::string& path)
{
    const fs::path dir =
         fs::path(toNative(path)).parent_path();
    return addTrailingSlash(dir.u8string());
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
    return fs::is_directory(toNative(path));
}

// Determine if the path is an absolute path
bool isAbsolutePath(const std::string& path)
{
    if (path.find("://") != std::string::npos)
        return true;

    return fs::path(toNative(path)).is_absolute();
}


void fileTimes(const std::string& filename, struct tm *createTime,
    struct tm *modTime)
{
#ifdef _WIN32
    struct _stat statbuf;
#ifdef PDAL_WIN32_STL
    std::wstring const wfilename(toNative(filename));
    _wstat(wfilename.c_str(), &statbuf);
#else
    std::string const wfilename(toNative(filename));
    _stat(wfilename.c_str(), &statbuf);
#endif

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

    if (path[0] == '~')
        throw pdal::pdal_error("PDAL does not support shell expansion");

#ifdef _WIN32
#ifdef PDAL_WIN32_STL
    std::wstring wpath(toNative(path));
    WIN32_FIND_DATAW ffd;
    HANDLE handle = FindFirstFileW(wpath.c_str(), &ffd);

    if (INVALID_HANDLE_VALUE == handle)
        return filenames;

    size_t found = wpath.find_last_of(L"/\\");
    do
    {
        // Ignore files starting with '.' to be consistent with UNIX.
        if (ffd.cFileName[0] == L'.')
            continue;
        if (found == std::wstring::npos)
            filenames.push_back(fromNative(ffd.cFileName));
        else
            filenames.push_back(fromNative(wpath.substr(0, found + 1)) +
                fromNative(ffd.cFileName));

    } while (FindNextFileW(handle, &ffd) != 0);
    FindClose(handle);
#else
    WIN32_FIND_DATA ffd;
    HANDLE handle = FindFirstFileA(path.c_str(), &ffd);

    if (INVALID_HANDLE_VALUE == handle)
        return filenames;

    size_t found = path.find_last_of("/\\");
    do
    {
        // Ignore files starting with '.' to be consistent with UNIX.
        if (ffd.cFileName[0] == '.')
            continue;
        if (found == std::wstring::npos)
            filenames.push_back(ffd.cFileName);
        else
            filenames.push_back(path.substr(0, found + 1) + ffd.cFileName);

    } while (FindNextFileA(handle, &ffd) != 0);
    FindClose(handle);
#endif
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


MapContext mapFile(const std::string& filename, bool readOnly, uintmax_t pos, uintmax_t size)
{
    MapContext ctx;

    if (!readOnly)
    {
        ctx.m_error = "readOnly must be true.";
        return ctx;
    }

    if (size == 0)
    {
        size = FileUtils::fileSize(filename);
        if (size == 0)
        {
            ctx.m_error = "File doesn't exist or isn't a regular file. Perhaps provide a size?";
            return ctx;
        }
    }

#ifndef PDAL_WIN32_STL
    ctx.m_fd = ::open(filename.c_str(), readOnly ? O_RDONLY : O_RDWR);
#else
    ctx.m_fd = ::_wopen(toNative(filename).data(), readOnly ? O_RDONLY : O_RDWR);
#endif

    if (ctx.m_fd == -1)
    {
        ctx.m_error = "Mapped file couldn't be opened.";
        return ctx;
    }
    ctx.m_size = size;

#ifndef _WIN32
    ctx.m_addr = ::mmap(0, size, PROT_READ, MAP_SHARED, ctx.m_fd, (off_t)pos);
    if (ctx.m_addr == MAP_FAILED)
    {
        ctx.m_addr = nullptr;
        ctx.m_error = "Couldn't map file";
    }
#else
    ctx.m_handle = CreateFileMapping((HANDLE)_get_osfhandle(ctx.m_fd),
        NULL, PAGE_READONLY, 0, 0, NULL);
    uint32_t low = pos & 0xFFFFFFFF;
    uint32_t high = (uint32_t)(pos >> 8);
    ctx.m_addr = MapViewOfFile(ctx.m_handle, FILE_MAP_READ, high, low,
        ctx.m_size);
    if (ctx.m_addr == nullptr)
        ctx.m_error = "Couldn't map file";
#endif

    return ctx;
}

MapContext unmapFile(MapContext ctx)
{
    if (!ctx.m_addr)
    {
    ctx.m_error = "File not mapped.";
        return ctx;
    }
#ifndef _WIN32
    if (::munmap(ctx.m_addr, ctx.m_size) == -1)
        ctx.m_error = "Couldn't unmap file.";
    else
    {
        ctx.m_addr = nullptr;
        ctx.m_size = 0;
        ctx.m_error = "";
    }
    ::close(ctx.m_fd);
#else
    if (UnmapViewOfFile(ctx.m_addr) == 0)
        ctx.m_error = "Couldn't unmap file.";
    else
    {
        ctx.m_addr = nullptr;
        ctx.m_size = 0;
        ctx.m_error = "";
    }
    CloseHandle(ctx.m_handle);
    ::_close(ctx.m_fd);
#endif
    return ctx;
}

} // namespace FileUtils
} // namespace pdal

