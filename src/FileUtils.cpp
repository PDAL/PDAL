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

namespace pdal
{


std::istream* FileUtils::openFile(std::string const& filename, bool asBinary)
{
    if (boost::algorithm::ifind_first(filename, "STDIN"))
        return &std::cin;

    if (!FileUtils::fileExists(filename))
        throw pdal_error("File not found: " + filename);

    std::ios::openmode mode = std::ios::in;
    if (asBinary)
        mode |= std::ios::binary;

    namespace io = boost::iostreams;
    io::stream<io::file_source>* ifs = new io::stream<io::file_source>();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
}


std::ostream* FileUtils::createFile(std::string const& filename, bool asBinary)
{
    std::ios::openmode mode = std::ios::out;
    if (asBinary)
        mode  |= std::ios::binary;

    if (boost::algorithm::ifind_first(filename, "STDOUT"))
        return &std::cout;


    namespace io = boost::iostreams;
    io::stream<io::file_sink>* ofs = new io::stream<io::file_sink>();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
}


void FileUtils::closeFile(std::ostream* ofs)
{
    namespace io = boost::iostreams;

    // An ofstream is closeable and deletable, but
    // an ostream like &std::cout isn't.
    if (!ofs) return;
    io::stream<io::file_sink>* sink = dynamic_cast<io::stream<io::file_sink>*>(ofs);
    if (sink)
    {
        sink->close();
        delete sink;
    }
}


void FileUtils::closeFile(std::istream* ifs)
{
    namespace io = boost::iostreams;

    // An ifstream is closeable and deletable, but
    // an istream like &std::cin isn't.
    if (!ifs) return;
    io::stream<io::file_source>* source = dynamic_cast<io::stream<io::file_source>*>(ifs);
    if (source)
    {
        source->close();
        delete source;
    }
}


bool FileUtils::deleteFile(const std::string& file)
{
    if (!fileExists(file))
        return false;

    return boost::filesystem::remove(file);
}


void FileUtils::renameFile(const std::string& dest, const std::string& src)
{
    boost::filesystem::rename(src, dest);
}


bool FileUtils::fileExists(const std::string& name)
{
    return boost::filesystem::exists(name) || boost::algorithm::iequals(name, "STDIN");
}


boost::uintmax_t FileUtils::fileSize(const std::string& file)
{
    return boost::filesystem::file_size(file);
}


std::string FileUtils::readFileIntoString(const std::string& filename)
{
    std::istream* stream = FileUtils::openFile(filename, false);
    assert(stream);
    std::string str((std::istreambuf_iterator<char>(*stream)), std::istreambuf_iterator<char>());
    FileUtils::closeFile(stream);
    return str;
}


std::string FileUtils::addTrailingSlash(const std::string& path)
{
    std::string ret = path;

    if (ret[ret.size() - 1] != '/')
        ret += "/";

    return ret;
}


std::string FileUtils::getcwd()
{
    const boost::filesystem::path p = boost::filesystem::current_path();
    std::string path = p.string();
    path = addTrailingSlash(path);
    return path;
}


// if the filename is an absolute path, just return it
// otherwise, make it absolute (relative to current working dir) and return that
std::string FileUtils::toAbsolutePath(const std::string& filename)
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
// note: if base dir is not absolute, first make it absolute via toAbsolutePath(base)
std::string FileUtils::toAbsolutePath(const std::string& filename, const std::string base)
{
    const std::string newbase = toAbsolutePath(base);

#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
    const boost::filesystem::path p = boost::filesystem::absolute(filename, newbase);
#else
    const boost::filesystem::path p = boost::filesystem::complete(filename, newbase);
#endif

    return p.string();
}


std::string FileUtils::getDirectory(const std::string& path)
{
    const boost::filesystem::path dir = boost::filesystem::path(path).parent_path();
    std::string ret = dir.string();
    ret = addTrailingSlash(ret);
    return ret;
}


bool FileUtils::isAbsolutePath(const std::string& path)
{
#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
    return boost::filesystem::path(path).is_absolute();
#else
    return boost::filesystem::path(path).is_complete();
#endif
}

std::string FileUtils::readFileAsString(std::string const& filename)
{
    if (!FileUtils::fileExists(filename))
    {
        std::ostringstream oss;
        oss << filename << " does not exist";
        throw pdal_error(oss.str());
    }

    std::istream::pos_type size;
    std::istream* input = FileUtils::openFile(filename, true);


    if (input->good())
    {
        std::string output;
        std::string line;
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
    else
    {
        FileUtils::closeFile(input);
        return std::string("");
    }

}


} // namespace pdal
