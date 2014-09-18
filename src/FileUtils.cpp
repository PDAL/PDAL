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

#include <pdal/FileUtils.hpp>

#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <sstream>

using namespace std;

namespace pdal
{


istream* FileUtils::openFile(string const& filename, bool asBinary)
{
    if (boost::algorithm::ifind_first(filename, "STDIN"))
        return &cin;

    if (!FileUtils::fileExists(filename))
        throw pdal_error("File not found: " + filename);

    ios::openmode mode = ios::in;
    if (asBinary)
        mode |= ios::binary;

    namespace io = boost::iostreams;
    io::stream<io::file_source>* ifs = new io::stream<io::file_source>();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
}


ostream* FileUtils::createFile(string const& filename, bool asBinary)
{
    ios::openmode mode = ios::out;
    if (asBinary)
        mode  |= ios::binary;

    if (boost::algorithm::ifind_first(filename, "STDOUT"))
        return &cout;


    namespace io = boost::iostreams;
    io::stream<io::file_sink>* ofs = new io::stream<io::file_sink>();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
}


void FileUtils::closeFile(ostream* ofs)
{
    namespace io = boost::iostreams;

    // An ofstream is closeable and deletable, but
    // an ostream like &std::cout isn't.
    if (!ofs) return;
    io::stream<io::file_sink>* sink =
        dynamic_cast<io::stream<io::file_sink>*>(ofs);
    if (sink)
    {
        sink->close();
        delete sink;
    }
}


void FileUtils::closeFile(istream* ifs)
{
    namespace io = boost::iostreams;

    // An ifstream is closeable and deletable, but
    // an istream like &std::cin isn't.
    if (!ifs)
        return;
    io::stream<io::file_source>* source =
        dynamic_cast<io::stream<io::file_source>*>(ifs);

    if (source)
    {
        source->close();
        delete source;
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
    return boost::filesystem::exists(name) ||
        boost::algorithm::iequals(name, "STDIN");
}

boost::uintmax_t FileUtils::fileSize(const string& file)
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
    if (path[path.size() - 1] != '/')
        path += "/";
    return path;
}


string FileUtils::getcwd()
{
    const boost::filesystem::path p = boost::filesystem::current_path();
    return addTrailingSlash(p.string());
}


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
