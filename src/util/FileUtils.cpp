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

#include <boost/filesystem.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_types.hpp>

using namespace std;

namespace pdal
{

namespace
{

bool isStdin(string filename)
{
    return Utils::toupper(filename) == "STDIN";
}

bool isStdout(string filename)
{
    return Utils::toupper(filename) == "STOUT" ||
        Utils::toupper(filename) == "STDOUT";
}

string addTrailingSlash(string path)
{
    if (path[path.size() - 1] != '/' && path[path.size() - 1] != '\\')
        path += "/";
    return path;
}

} // unnamed namespace

namespace FileUtils
{

istream *openFile(string const& filename, bool asBinary)
{
    if (isStdin(filename))
        return &cin;

    if (!FileUtils::fileExists(filename))
        return NULL;

    ios::openmode mode = ios::in;
    if (asBinary)
        mode |= ios::binary;

    ifstream *ifs = new ifstream(filename, mode);
    if (!ifs->good())
    {
        delete ifs;
        return NULL;
    }
    return ifs;
}


ostream *createFile(string const& filename, bool asBinary)
{
    if (isStdout(filename))
        return &cout;

    ios::openmode mode = ios::out;
    if (asBinary)
        mode |= ios::binary;

    ostream *ofs = new ofstream(filename, mode);
    if (! ofs->good())
    {
        delete ofs;
        return NULL;
    }
    return ofs;
}


bool directoryExists(const string& dirname)
{
    return pdalboost::filesystem::exists(dirname);
}


bool createDirectory(const string& dirname)
{
    return pdalboost::filesystem::create_directory(dirname);
}


void deleteDirectory(const string& dirname)
{
    pdalboost::filesystem::remove_all(dirname);
}


StringList directoryList(const string& dir)
{
    StringList files;

    pdalboost::filesystem::directory_iterator it(dir);
    pdalboost::filesystem::directory_iterator end;
    while (it != end)
    {
        files.push_back(it->path().string());
        it++;
    }
    return files;
}


void closeFile(ostream *out)
{
    // An ofstream is closeable and deletable, but
    // an ostream like &cout isn't.
    if (!out)
        return;
    ofstream *ofs = dynamic_cast<ofstream *>(out);
    if (ofs)
    {
        ofs->close();
        delete ofs;
    }
}


void closeFile(istream* in)
{
    // An ifstream is closeable and deletable, but
    // an istream like &cin isn't.
    if (!in)
        return;
    ifstream *ifs = dynamic_cast<ifstream *>(in);
    if (ifs)
    {
        ifs->close();
        delete ifs;
    }
}


bool deleteFile(const string& file)
{
    if (!fileExists(file))
        return false;

    return pdalboost::filesystem::remove(file);
}


void renameFile(const string& dest, const string& src)
{
    pdalboost::filesystem::rename(src, dest);
}


bool fileExists(const string& name)
{
    // filename may actually be a greyhound uri + pipelineId
    string http = name.substr(0, 4);
    if (Utils::iequals(http, "http"))
        return true;

    pdalboost::system::error_code ec;
    pdalboost::filesystem::exists(name, ec);
    return pdalboost::filesystem::exists(name) || isStdin(name);
}


uintmax_t fileSize(const string& file)
{
    return pdalboost::filesystem::file_size(file);
}


string readFileIntoString(const string& filename)
{
    istream* stream = openFile(filename, false);
    assert(stream);
    string str((istreambuf_iterator<char>(*stream)),
        istreambuf_iterator<char>());
    closeFile(stream);
    return str;
}


string getcwd()
{
    const pdalboost::filesystem::path p = pdalboost::filesystem::current_path();
    return addTrailingSlash(p.string());
}


/***
// Non-boost alternative.  Requires file existence.
string toAbsolutePath(const string& filename)
{
    string result;

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
string toAbsolutePath(const string& filename)
{
    return pdalboost::filesystem::absolute(filename).string();
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to base dir) and return that
//
// note: if base dir is not absolute, first make it absolute via
// toAbsolutePath(base)
string toAbsolutePath(const string& filename, const string base)
{
    const string newbase = toAbsolutePath(base);
    return pdalboost::filesystem::absolute(filename, newbase).string();
}

string getFilename(const string& path)
{
#ifdef _WIN32
    std::string pathsep("\\/");
#else
    char pathsep = '/';
#endif

    string::size_type pos = path.find_last_of(pathsep);
    if (pos == string::npos)
        return path;
    return path.substr(pos + 1);
}


// Get the directory part of a filename.
string getDirectory(const string& path)
{
    const pdalboost::filesystem::path dir =
         pdalboost::filesystem::path(path).parent_path();
    return addTrailingSlash(dir.string());
}


string stem(const string& path)
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
bool isAbsolutePath(const string& path)
{
    return pdalboost::filesystem::path(path).is_absolute();
}


void fileTimes(const string& filename, struct tm *createTime,
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

} // namespace FileUtils

} // namespace pdal

