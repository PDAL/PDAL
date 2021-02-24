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
#include <algorithm>
#ifndef _WIN32
#include <glob.h>
#include <sys/mman.h>
#include <signal.h>
#include <thread>
#include <chrono>
#else
#include <io.h>
#include <codecvt>
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

#ifdef _WIN32
inline std::string fromNative(std::wstring const& in)
{
    // TODO: C++11 define convert with static thread_local
    std::wstring_convert<std::codecvt_utf8_utf16<unsigned short>, unsigned short> convert;
    auto p = reinterpret_cast<unsigned short const*>(in.data());
    return convert.to_bytes(p, p + in.size());
}
inline std::wstring toNative(std::string const& in)
{
    // TODO: C++11 define convert with static thread_local
    std::wstring_convert<std::codecvt_utf8_utf16<unsigned short>, unsigned short> convert;
    auto s = convert.from_bytes(in);
    auto p = reinterpret_cast<wchar_t const*>(s.data());
    return std::wstring(p, p + s.size());
}
#else
// inline std::string const& fromNative(std::string const& in) { return in; }
inline std::string const& toNative(std::string const& in) { return in; }
#endif

} // unnamed namespace

namespace FileUtils
{

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

    ifs = new std::ifstream(toNative(name), mode);
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

    std::ostream *ofs = new std::ofstream(toNative(name), mode);
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
    return pdalboost::filesystem::exists(toNative(dirname));
}


bool createDirectory(const std::string& dirname)
{
    return pdalboost::filesystem::create_directory(toNative(dirname));
}


bool createDirectories(const std::string& dirname)
{
    return pdalboost::filesystem::create_directories(toNative(dirname));
}


void deleteDirectory(const std::string& dirname)
{
    pdalboost::filesystem::remove_all(toNative(dirname));
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
    return pdalboost::filesystem::remove(toNative(file));
}


void renameFile(const std::string& dest, const std::string& src)
{
    pdalboost::filesystem::rename(toNative(src), toNative(dest));
}


bool fileExists(const std::string& name)
{
    if (isStdin(name))
        return true;

    try
    {
        return pdalboost::filesystem::exists(toNative(name));
    }
    catch (pdalboost::filesystem::filesystem_error&)
    {
    }
    return false;
}


uintmax_t fileSize(const std::string& file)
{
    pdalboost::system::error_code ec;
    return pdalboost::filesystem::file_size(toNative(file), ec);
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


std::string toCanonicalPath(std::string filename)
{
    std::string result;

#ifdef _WIN32
    filename = addTrailingSlash(filename);
    char buf[MAX_PATH];
    if (GetFullPathName(filename.c_str(), MAX_PATH, buf, NULL))
        result = buf;
#else
    char *buf = realpath(filename.c_str(), NULL);
    if (buf)
    {
        result = buf;
        free(buf);
    }
#endif
    return result;
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to current working dir) and return that
std::string toAbsolutePath(const std::string& filename)
{
    return pdalboost::filesystem::absolute(toNative(filename)).string();
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to base dir) and return that
//
// note: if base dir is not absolute, first make it absolute via
// toAbsolutePath(base)
std::string toAbsolutePath(const std::string& filename, const std::string base)
{
    const std::string newbase = toAbsolutePath(base);
    return pdalboost::filesystem::absolute(toNative(filename),
        toNative(newbase)).string();
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
    const pdalboost::filesystem::path dir =
         pdalboost::filesystem::path(toNative(path)).parent_path();
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
    return pdalboost::filesystem::is_directory(toNative(path));
}

// Determine if the path is an absolute path
bool isAbsolutePath(const std::string& path)
{
    return pdalboost::filesystem::path(toNative(path)).is_absolute();
}


void fileTimes(const std::string& filename, struct tm *createTime,
    struct tm *modTime)
{
#ifdef _WIN32
    std::wstring const wfilename(toNative(filename));
    struct _stat statbuf;
    _wstat(wfilename.c_str(), &statbuf);

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
            filenames.push_back(fromNative(wpath.substr(0, found)) + "\\" + fromNative(ffd.cFileName));

    } while (FindNextFileW(handle, &ffd) != 0);
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


std::string tmpDirectory()
{
    pdalboost::filesystem::path path = pdalboost::filesystem::temp_directory_path();
    path /= "pdal";
    if (!directoryExists(path.string()))
        createDirectory(path.string());
        
    return addTrailingSlash(path.string());
}

std::string tmpFileName(const std::string& extension)
{
    const size_t length = 16;
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string filename(length,0);
    std::generate_n( filename.begin(), length, randchar );

    if (!extension.empty()) //if user wants an extension
    {
        if (extension.rfind(".", 0) == 0)
           filename.append(extension);
        else
            filename.append(".").append(extension);
    }

    pdalboost::filesystem::path path = tmpDirectory();

    path /= filename;

    if (fileExists(path.string()))
       return tmpFileName(extension);

    return path.string();
}


MapContext mapFile(const std::string& filename, bool readOnly,
    size_t pos, size_t size)
{
    MapContext ctx;

    if (!readOnly)
    {
        ctx.m_error = "readOnly must be true.";
        return ctx;
    }

#ifndef WIN32
    ctx.m_fd = ::open(filename.c_str(), readOnly ? O_RDONLY : O_RDWR);
#else
    ctx.m_fd = ::_open(filename.c_str(), readOnly ? O_RDONLY : O_RDWR);
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
    uint32_t high = (pos >> 8);
    ctx.m_addr = MapViewOfFile(ctx.m_handle, FILE_MAP_READ, high, low,
        ctx.m_size);
    if (ctx.m_addr == nullptr)
        ctx.m_error = "Couldn't map file";
#endif

    return ctx;
}

MapContext unmapFile(MapContext ctx)
{
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

  
#ifdef  WIN32
 
LockFile::LockFile()
{
    m_mutex = INVALID_HANDLE_VALUE;
    m_flagged = false;
}
 
LockFile::LockFile(const char *name)
{
    m_mutex = INVALID_HANDLE_VALUE;
    m_flagged = false;
    lock(name);
}
 
bool LockFile::lock(const char *name)
{
    char mname[65];
    char *ext = strrchr((char *)name, '/');
 
    if(ext)
        name = ++ext;
 
    unlock();
    snprintf(mname, sizeof(mname) - 4, "_lock_%s", name);
    ext = strrchr(mname, '.');
    if(ext && !strcasecmp(ext, ".lock"))
    {
        *ext = 0;
        ext = NULL;
    }
    if(!ext)
        addString(mname, sizeof(mname), ".lck");
    m_mutex = CreateMutex(NULL, FALSE, mname);
    if(WaitForSingleObject(m_mutex, 200) == WAIT_OBJECT_0)
        m_flagged = true;
    return m_flagged;
}
 
void LockFile::unlock()
{
    if(m_mutex == INVALID_HANDLE_VALUE)
        return;
 
    if(m_flagged)
        ReleaseMutex(m_mutex);
    CloseHandle(m_mutex);
    m_flagged = false;
    m_mutex = INVALID_HANDLE_VALUE;
}
 
bool LockFile::isLocked()
{
    return m_flagged;
}
 
#else
 
LockFile::LockFile()
{
    m_path = NULL;
}
 
LockFile::LockFile(const char *name)
{
    m_path = NULL;
    lock(name);
}
 
bool LockFile::lock(const char *name)
{
    struct stat ino;
    int fd, pid, status;
    const char *ext;
    char buffer[128];
    bool rtn = true;
 
    unlock();
 
    ext = strrchr(name, '/');
    if(ext)
        ext = strrchr(ext, '.');
    else
        ext = strrchr(name, '.');
 
    if(strchr(name, '/'))
    {
        m_path = new char[strlen(name) + 1];
        strcpy(m_path, name);
    }
    else if(ext && !strcasecmp(ext, ".pid"))
    {
        if(stat("/var/run", &ino))
            snprintf(buffer, sizeof(buffer), "/tmp/.%s", name);
        else
            snprintf(buffer, sizeof(buffer), "/var/run/%s", name);
        m_path = new char[strlen(buffer) + 1];
        strcpy(m_path, buffer);
    }
    else
    {
        if(!ext)
            ext = ".lock";
        if(stat("/var/lock", &ino))
            snprintf(buffer, sizeof(buffer), "/tmp/.%s%s", name, ext);
        else
            snprintf(buffer, sizeof(buffer), "/var/lock/%s%s", name, ext);
 
        m_path = new char[strlen(buffer) + 1];
        strcpy(m_path, buffer);
    }
 
    for(;;)
    {
        fd = ::open(m_path, O_WRONLY | O_CREAT | O_EXCL, 0660);
        if(fd > 0)
        {
            pid = getpid();
            snprintf(buffer, sizeof(buffer), "%d\n", pid);
            if(::write(fd, buffer, strlen(buffer)))
                rtn = false;
            ::close(fd);
            return rtn;
        }
        if(fd < 0 && errno != EEXIST)
        {
            delete[] m_path;
            m_path = NULL;
            return false;
        }
 
        fd = ::open(m_path, O_RDONLY);
        if(fd < 0)
        {
            if(errno == ENOENT)
                continue;
            delete[] m_path;
            m_path = NULL;
            return false;
        }
 
        std::this_thread::sleep_for(std::chrono::seconds(1));
        status = ::read(fd, buffer, sizeof(buffer) - 1);
        if(status < 1) {
            ::close(fd);
            continue;
        }
 
        buffer[status] = 0;
        pid = atoi(buffer);
        if(pid)
        {
            if(pid == getpid())
            {
                status = -1;
                errno = 0;
            }
            else
                status = kill(pid, 0);
 
            if(!status || (errno == EPERM))
            {
                ::close(fd);
                delete[] m_path;
                m_path = NULL;
                return false;
            }
        }
        ::close(fd);
        ::unlink(m_path);
    }
}
 
void LockFile::unlock()
{
    if(m_path)
    {
        pdalboost::filesystem::remove(toNative(m_path));
        delete[] m_path;
        m_path = NULL;
    }
}
 
bool LockFile::isLocked()
{
    if(m_path)
        return true;
 
    return false;
}
 
#endif


} // namespace FileUtils
} // namespace pdal

