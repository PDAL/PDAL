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

using namespace std;

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

} // unnamed namespace

istream* FileUtils::openFile(string const& filename, bool asBinary)
{
    if (isStdin(filename))
        return &cin;

    if (!FileUtils::fileExists(filename))
        throw pdal_error(std::string("File '") + filename + "' not found");

    ios::openmode mode = ios::in;
    if (asBinary)
        mode |= ios::binary;

    std::ifstream *ifs = new std::ifstream(filename, mode);
    if (!ifs->good())
    {
        delete ifs;
        return NULL;
    }
    return ifs;
}


ostream* FileUtils::createFile(string const& filename, bool asBinary)
{
    if (isStdout(filename))
        return &cout;

    ios::openmode mode = ios::out;
    if (asBinary)
        mode |= ios::binary;

    std::ostream *ofs = new std::ofstream(filename, mode);
    if (! ofs->good())
    {
        delete ofs;
        return NULL;
    }
    return ofs;
}


bool FileUtils::directoryExists(std::string const& dirname)
{
    return boost::filesystem::exists(dirname);
}


bool FileUtils::createDirectory(std::string const& dirname)
{
    return boost::filesystem::create_directory(dirname); 
}


void FileUtils::deleteDirectory(std::string const& dirname)
{
    boost::filesystem::remove_all(dirname);
}


void FileUtils::closeFile(ostream *out)
{
    // An ofstream is closeable and deletable, but
    // an ostream like &std::cout isn't.
    if (!out)
        return;
    ofstream *ofs = dynamic_cast<ofstream *>(out);
    if (ofs)
    {
        ofs->close();
        delete ofs;
    }
}


void FileUtils::closeFile(istream* in)
{
    // An ifstream is closeable and deletable, but
    // an istream like &std::cin isn't.
    if (!in)
        return;
    ifstream *ifs = dynamic_cast<ifstream *>(in);
    if (ifs)
    {
        ifs->close();
        delete ifs;
    }
}


bool FileUtils::deleteFile(const string& file)
{
    if (!fileExists(file))
        return false;

    return boost::filesystem::remove(file);
}


void FileUtils::renameFile(const string& dest, const string& src)
{
    boost::filesystem::rename(src, dest);
}


bool FileUtils::fileExists(const string& name)
{
    // filename may actually be a greyhound uri + pipelineId
    std::string http = name.substr(0, 4);
    if (Utils::iequals(http, "http"))
        return true;
 
    boost::system::error_code ec;
    boost::filesystem::exists(name, ec);
    return boost::filesystem::exists(name) || isStdin(name);
}


uintmax_t FileUtils::fileSize(const string& file)
{
    return boost::filesystem::file_size(file);
}


string FileUtils::readFileIntoString(const string& filename)
{
    istream* stream = FileUtils::openFile(filename, false);
    assert(stream);
    string str((istreambuf_iterator<char>(*stream)),
        istreambuf_iterator<char>());
    FileUtils::closeFile(stream);
    return str;
}


string FileUtils::addTrailingSlash(string path)
{
    if (path[path.size() - 1] != '/' && path[path.size() - 1] != '\\')
        path += "/";
    return path;
}


string FileUtils::getcwd()
{
    const boost::filesystem::path p = boost::filesystem::current_path();
    return addTrailingSlash(p.string());
}


/***
// Non-boost alternative.  Requires file existence.
string FileUtils::toAbsolutePath(const string& filename)
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
string FileUtils::toAbsolutePath(const string& filename)
{

#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
    const boost::filesystem::path p = boost::filesystem::absolute(filename);
#else
    const boost::filesystem::path p = boost::filesystem::complete(filename);
#endif

    return p.string();
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to base dir) and return that
//
// note: if base dir is not absolute, first make it absolute via
// toAbsolutePath(base)
string FileUtils::toAbsolutePath(const string& filename, const string base)
{
    const string newbase = toAbsolutePath(base);

#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
    const boost::filesystem::path p = boost::filesystem::absolute(filename, newbase);
#else
    const boost::filesystem::path p = boost::filesystem::complete(filename, newbase);
#endif

    return p.string();
}

string FileUtils::getFilename(const string& path)
{
#ifdef _WIN32
    char pathsep = '\\';
#else
    char pathsep = '/';
#endif

    std::string::size_type pos = path.find_last_of(pathsep);
    if (pos == std::string::npos)
        return path;
    return path.substr(pos + 1);
}

// Get the directory part of a filename.
string FileUtils::getDirectory(const string& path)
{
    const boost::filesystem::path dir =
         boost::filesystem::path(path).parent_path();
    return addTrailingSlash(dir.string());
}


// Determine if the path is an absolute path
bool FileUtils::isAbsolutePath(const string& path)
{
#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
    return boost::filesystem::path(path).is_absolute();
#else
    return boost::filesystem::path(path).is_complete();
#endif
}


void FileUtils::fileTimes(const std::string& filename,
    struct tm *createTime, struct tm *modTime)
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


string FileUtils::readFileAsString(string const& filename)
{
    if (!FileUtils::fileExists(filename))
    {
        ostringstream oss;
        oss << filename << " does not exist";
        throw pdal_error(oss.str());
    }

    istream::pos_type size;
    istream* input = FileUtils::openFile(filename, true);

    if (input->good())
    {
        string output;
        string line;
        while (input->good())
        {
            getline(*input, line);
            if (output.size())
            {
                output = output + "\n" + line;
            }
            else
            {
                output = line;
            }
        }
        return output;
    }
    FileUtils::closeFile(input);
    return string();
}

} // namespace pdal
