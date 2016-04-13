/// Arbiter amalgamated source (https://github.com/connormanning/arbiter).
/// It is intended to be used with #include "arbiter.hpp"

// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: LICENSE
// //////////////////////////////////////////////////////////////////////

/*
The MIT License (MIT)

Copyright (c) 2015 Connor Manning

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


*/

// //////////////////////////////////////////////////////////////////////
// End of content of file: LICENSE
// //////////////////////////////////////////////////////////////////////






#include "arbiter.hpp"

#ifndef ARBITER_IS_AMALGAMATION
#error "Compile with -I PATH_TO_ARBITER_DIRECTORY"
#endif


// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/arbiter.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>

#include <arbiter/driver.hpp>
#endif

#include <algorithm>
#include <cstdlib>

namespace arbiter
{

namespace
{
    const std::string delimiter("://");

    const std::size_t concurrentHttpReqs(32);
    const std::size_t httpRetryCount(8);
}

Arbiter::Arbiter()
    : m_drivers()
    , m_pool(concurrentHttpReqs, httpRetryCount, Json::Value())
{
    init(Json::Value());
}

Arbiter::Arbiter(const Json::Value& json)
    : m_drivers()
    , m_pool(concurrentHttpReqs, httpRetryCount, json)
{
    init(json);
}

void Arbiter::init(const Json::Value& json)
{
    using namespace drivers;

    auto fs(Fs::create(m_pool, json["file"]));
    if (fs) m_drivers["file"] = std::move(fs);

    auto http(Http::create(m_pool, json["http"]));
    if (http) m_drivers["http"] = std::move(http);

    auto s3(S3::create(m_pool, json["s3"]));
    if (s3) m_drivers["s3"] = std::move(s3);

    auto dropbox(Dropbox::create(m_pool, json["dropbox"]));
    if (dropbox) m_drivers["dropbox"] = std::move(dropbox);
}

void Arbiter::addDriver(const std::string type, std::unique_ptr<Driver> driver)
{
    if (!driver) throw ArbiterError("Cannot add empty driver for " + type);
    m_drivers[type] = std::move(driver);
}

std::string Arbiter::get(const std::string path) const
{
    return getDriver(path).get(stripType(path));
}

std::vector<char> Arbiter::getBinary(const std::string path) const
{
    return getDriver(path).getBinary(stripType(path));
}

std::unique_ptr<std::string> Arbiter::tryGet(std::string path) const
{
    return getDriver(path).tryGet(stripType(path));
}

std::unique_ptr<std::vector<char>> Arbiter::tryGetBinary(std::string path) const
{
    return getDriver(path).tryGetBinary(stripType(path));
}

std::size_t Arbiter::getSize(const std::string path) const
{
    return getDriver(path).getSize(stripType(path));
}

std::unique_ptr<std::size_t> Arbiter::tryGetSize(const std::string path) const
{
    return getDriver(path).tryGetSize(stripType(path));
}

void Arbiter::put(const std::string path, const std::string& data) const
{
    return getDriver(path).put(stripType(path), data);
}

void Arbiter::put(const std::string path, const std::vector<char>& data) const
{
    return getDriver(path).put(stripType(path), data);
}

void Arbiter::copy(const std::string from, const std::string to) const
{
    const Endpoint outEndpoint(getEndpoint(to));
    const auto paths(resolve(from));

    for (const auto& path : paths)
    {
        outEndpoint.putSubpath(getTerminus(path), getBinary(path));
    }
}

bool Arbiter::isRemote(const std::string path) const
{
    return getDriver(path).isRemote();
}

bool Arbiter::isLocal(const std::string path) const
{
    return !isRemote(path);
}

std::vector<std::string> Arbiter::resolve(
        const std::string path,
        const bool verbose) const
{
    return getDriver(path).resolve(stripType(path), verbose);
}

Endpoint Arbiter::getEndpoint(const std::string root) const
{
    return Endpoint(getDriver(root), stripType(root));
}

const Driver& Arbiter::getDriver(const std::string path) const
{
    const auto type(getType(path));

    if (!m_drivers.count(type))
    {
        throw ArbiterError("No driver for " + path);
    }

    return *m_drivers.at(type);
}

std::unique_ptr<fs::LocalHandle> Arbiter::getLocalHandle(
        const std::string path,
        const Endpoint& tempEndpoint) const
{
    std::unique_ptr<fs::LocalHandle> localHandle;

    if (isRemote(path))
    {
        if (tempEndpoint.isRemote())
        {
            throw ArbiterError("Temporary endpoint must be local.");
        }

        std::string name(path);
        std::replace(name.begin(), name.end(), '/', '-');
        std::replace(name.begin(), name.end(), '\\', '-');
        std::replace(name.begin(), name.end(), ':', '_');

        tempEndpoint.putSubpath(name, getBinary(path));

        localHandle.reset(
                new fs::LocalHandle(tempEndpoint.root() + name, true));
    }
    else
    {
        localHandle.reset(
                new fs::LocalHandle(fs::expandTilde(stripType(path)), false));
    }

    return localHandle;
}

std::unique_ptr<fs::LocalHandle> Arbiter::getLocalHandle(
        const std::string path,
        std::string tempPath) const
{
    if (tempPath.empty()) tempPath = fs::getTempPath();
    return getLocalHandle(path, getEndpoint(tempPath));
}

std::string Arbiter::getType(const std::string path)
{
    std::string type("file");
    const std::size_t pos(path.find(delimiter));

    if (pos != std::string::npos)
    {
        type = path.substr(0, pos);
    }

    return type;
}

std::string Arbiter::stripType(const std::string raw)
{
    std::string result(raw);
    const std::size_t pos(raw.find(delimiter));

    if (pos != std::string::npos)
    {
        result = raw.substr(pos + delimiter.size());
    }

    return result;
}

std::string Arbiter::getTerminus(const std::string fullPath)
{
    std::string result(fullPath);

    std::string stripped(stripType(fullPath));

    for (std::size_t i(0); i < 2; ++i)
    {
        // Pop trailing asterisk, or double-trailing-asterisks for both non- and
        // recursive globs.
        if (!stripped.empty() && stripped.back() == '*') stripped.pop_back();
    }

    // Pop trailing slash, in which case the result is the innermost directory.
    if (!stripped.empty() && stripped.back() == '/') stripped.pop_back();

    // Now do the real slash searching.
    const std::size_t pos(stripped.rfind('/'));

    if (pos != std::string::npos)
    {
        const std::string sub(stripped.substr(pos));
        if (!sub.empty()) result = sub;
    }

    return result;
}

} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/arbiter.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/driver.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/driver.hpp>

#include <arbiter/arbiter.hpp>
#endif

namespace arbiter
{

std::unique_ptr<std::vector<char>> Driver::tryGetBinary(std::string path) const
{
    std::unique_ptr<std::vector<char>> data(new std::vector<char>());
    if (!get(path, *data)) data.reset();
    return data;
}

std::vector<char> Driver::getBinary(std::string path) const
{
    std::vector<char> data;
    if (!get(path, data)) throw ArbiterError("Could not read file " + path);
    return data;
}

std::unique_ptr<std::string> Driver::tryGet(const std::string path) const
{
    std::unique_ptr<std::string> result;
    std::unique_ptr<std::vector<char>> data(tryGetBinary(path));
    if (data) result.reset(new std::string(data->begin(), data->end()));
    return result;
}

std::string Driver::get(const std::string path) const
{
    const std::vector<char> data(getBinary(path));
    return std::string(data.begin(), data.end());
}

std::size_t Driver::getSize(const std::string path) const
{
    if (auto size = tryGetSize(path)) return *size;
    else throw ArbiterError("Could not get size of " + path);
}

void Driver::put(std::string path, const std::string& data) const
{
    put(path, std::vector<char>(data.begin(), data.end()));
}

std::vector<std::string> Driver::resolve(
        std::string path,
        const bool verbose) const
{
    std::vector<std::string> results;

    if (path.size() > 1 && path.back() == '*')
    {
        if (verbose)
        {
            std::cout << "Resolving [" << type() << "]: " << path << " ..." <<
                std::flush;
        }

        results = glob(path, verbose);

        if (verbose)
        {
            std::cout << "\n\tResolved to " << results.size() <<
                " paths." << std::endl;
        }
    }
    else
    {
        if (type() != "fs") path = type() + "://" + path;

        results.push_back(path);
    }

    return results;
}

std::vector<std::string> Driver::glob(std::string path, bool verbose) const
{
    throw ArbiterError("Cannot glob driver for: " + path);
}

} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/driver.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/endpoint.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/endpoint.hpp>

#include <arbiter/arbiter.hpp>
#include <arbiter/driver.hpp>
#endif

namespace arbiter
{

namespace
{
    std::string postfixSlash(std::string path)
    {
        if (path.empty()) throw ArbiterError("Invalid root path");
        if (path.back() != '/') path.push_back('/');
        return path;
    }
}

Endpoint::Endpoint(const Driver& driver, const std::string root)
    : m_driver(driver)
    , m_root(fs::expandTilde(postfixSlash(root)))
{ }

std::string Endpoint::root() const
{
    return m_root;
}

std::string Endpoint::type() const
{
    return m_driver.type();
}

bool Endpoint::isRemote() const
{
    return m_driver.isRemote();
}

bool Endpoint::isLocal() const
{
    return !isRemote();
}

std::string Endpoint::getSubpath(const std::string subpath) const
{
    return m_driver.get(fullPath(subpath));
}

std::unique_ptr<std::string> Endpoint::tryGetSubpath(const std::string subpath)
    const
{
    return m_driver.tryGet(fullPath(subpath));
}

std::vector<char> Endpoint::getSubpathBinary(const std::string subpath) const
{
    return m_driver.getBinary(fullPath(subpath));
}

std::unique_ptr<std::vector<char>> Endpoint::tryGetSubpathBinary(
        const std::string subpath) const
{
    return m_driver.tryGetBinary(fullPath(subpath));
}

void Endpoint::putSubpath(
        const std::string subpath,
        const std::string& data) const
{
    m_driver.put(fullPath(subpath), data);
}

void Endpoint::putSubpath(
        const std::string subpath,
        const std::vector<char>& data) const
{
    m_driver.put(fullPath(subpath), data);
}

std::string Endpoint::fullPath(const std::string& subpath) const
{
    return m_root + subpath;
}

Endpoint Endpoint::getSubEndpoint(std::string subpath) const
{
    return Endpoint(m_driver, m_root + subpath);
}

} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/endpoint.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/fs.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#endif

#ifndef ARBITER_WINDOWS
#include <glob.h>
#include <sys/stat.h>
#else
#include <locale>
#include <codecvt>
#endif

#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace arbiter
{

namespace
{
    // Binary output, overwriting any existing file with a conflicting name.
    const std::ios_base::openmode binaryTruncMode(
            std::ofstream::binary |
            std::ofstream::out |
            std::ofstream::trunc);

    void noHome()
    {
        throw ArbiterError("No home directory found");
    }
}

namespace drivers
{

std::unique_ptr<Fs> Fs::create(HttpPool&, const Json::Value&)
{
    return std::unique_ptr<Fs>(new Fs());
}

std::unique_ptr<std::size_t> Fs::tryGetSize(std::string path) const
{
    std::unique_ptr<std::size_t> size;

    path = fs::expandTilde(path);

    std::ifstream stream(path, std::ios::in | std::ios::binary);

    if (stream.good())
    {
        stream.seekg(0, std::ios::end);
        size.reset(new std::size_t(stream.tellg()));
    }

    return size;
}

bool Fs::get(std::string path, std::vector<char>& data) const
{
    bool good(false);

    path = fs::expandTilde(path);
    std::ifstream stream(path, std::ios::in | std::ios::binary);

    if (stream.good())
    {
        stream.seekg(0, std::ios::end);
        data.resize(static_cast<std::size_t>(stream.tellg()));
        stream.seekg(0, std::ios::beg);
        stream.read(data.data(), data.size());
        stream.close();
        good = true;
    }

    return good;
}

void Fs::put(std::string path, const std::vector<char>& data) const
{
    path = fs::expandTilde(path);
    std::ofstream stream(path, binaryTruncMode);

    if (!stream.good())
    {
        throw ArbiterError("Could not open " + path + " for writing");
    }

    stream.write(data.data(), data.size());

    if (!stream.good())
    {
        throw ArbiterError("Error occurred while writing " + path);
    }
}

std::vector<std::string> Fs::glob(std::string path, bool) const
{
    std::vector<std::string> results;

#ifndef ARBITER_WINDOWS
    path = fs::expandTilde(path);

    glob_t buffer;
    struct stat info;

    ::glob(path.c_str(), GLOB_NOSORT | GLOB_TILDE, 0, &buffer);

    for (std::size_t i(0); i < buffer.gl_pathc; ++i)
    {
        const std::string val(buffer.gl_pathv[i]);

        if (stat(val.c_str(), &info) == 0)
        {
            if (S_ISREG(info.st_mode))
            {
                results.push_back(val);
            }
        }
        else
        {
            throw ArbiterError("Error globbing - POSIX stat failed");
        }
    }

    globfree(&buffer);
#else
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    const std::wstring wide(converter.from_bytes(path));

    WIN32_FIND_DATA data;
    HANDLE hFind(FindFirstFile(wide.c_str(), &data));

    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
            {
                results.push_back(converter.to_bytes(data.cFileName));
            }
        }
        while (FindNextFile(hFind, &data));
    }
#endif

    return results;
}

} // namespace drivers

namespace fs
{

bool mkdirp(std::string dir)
{
    dir = expandTilde(dir);

#ifndef ARBITER_WINDOWS
    const bool err(::mkdir(dir.c_str(), S_IRWXU | S_IRGRP | S_IROTH));
    return (!err || errno == EEXIST);
#else
    throw ArbiterError("Windows mkdirp not done yet.");
#endif
}

bool remove(std::string filename)
{
    filename = expandTilde(filename);

#ifndef ARBITER_WINDOWS
    return ::remove(filename.c_str()) == 0;
#else
    throw ArbiterError("Windows remove not done yet.");
#endif
}

std::string expandTilde(std::string in)
{
    std::string out(in);

    if (!in.empty() && in.front() == '~')
    {
#ifndef ARBITER_WINDOWS
        if (!getenv("HOME"))
        {
            noHome();
        }

        static const std::string home(getenv("HOME"));
#else
        if (
                !getenv("USERPROFILE") &&
                !(getenv("HOMEDRIVE") && getenv("HOMEPATH")))
        {
            noHome();
        }

        static const std::string home(
                getenv("USERPROFILE") ? getenv("USERPROFILE") :
                    (getenv("HOMEDRIVE") + getenv("HOMEPATH"));
#endif

        out = home + in.substr(1);
    }

    return out;
}

std::string getTempPath()
{
    std::string result;

#ifndef ARBITER_WINDOWS
    if (const char* t = getenv("TMPDIR"))   return t;
    if (const char* t = getenv("TMP"))      return t;
    if (const char* t = getenv("TEMP"))     return t;
    if (const char* t = getenv("TEMPDIR"))  return t;
    if (result.empty()) return "/tmp";
#else
    throw ArbiterError("Windows getTempPath not done yet.");
#endif

    return result;
}

LocalHandle::LocalHandle(const std::string localPath, const bool isRemote)
    : m_localPath(expandTilde(localPath))
    , m_erase(isRemote)
{ }

LocalHandle::~LocalHandle()
{
    if (m_erase) fs::remove(fs::expandTilde(m_localPath));
}

} // namespace fs
} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/fs.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/http.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/http.hpp>
#endif

#ifdef ARBITER_WINDOWS
#undef min
#undef max
#endif

#include <algorithm>
#include <cstring>
#include <iostream>

namespace
{
    struct PutData
    {
        PutData(const std::vector<char>& data)
            : data(data)
            , offset(0)
        { }

        const std::vector<char>& data;
        std::size_t offset;
    };

    std::size_t getCb(
            const char* in,
            std::size_t size,
            std::size_t num,
            std::vector<char>* out)
    {
        const std::size_t fullBytes(size * num);
        const std::size_t startSize(out->size());

        out->resize(out->size() + fullBytes);
        std::memcpy(out->data() + startSize, in, fullBytes);

        return fullBytes;
    }

    std::size_t putCb(
            char* out,
            std::size_t size,
            std::size_t num,
            PutData* in)
    {
        const std::size_t fullBytes(
                std::min(
                    size * num,
                    in->data.size() - in->offset));
        std::memcpy(out, in->data.data() + in->offset, fullBytes);

        in->offset += fullBytes;
        return fullBytes;
    }

    std::size_t headerCb(
            const char *buffer,
            std::size_t size,
            std::size_t num,
            arbiter::Headers* out)
    {
        const std::size_t fullBytes(size * num);

        std::string data(buffer, fullBytes);
        data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());
        data.erase(std::remove(data.begin(), data.end(), '\r'), data.end());

        const std::size_t split(data.find_first_of(":"));

        // No colon means it isn't a header with data.
        if (split == std::string::npos) return fullBytes;

        const std::string key(data.substr(0, split));
        const std::string val(data.substr(split + 1, data.size()));

        (*out)[key] = val;

        return fullBytes;
    }

    size_t eatLogging(void *out, size_t size, size_t num, void *in)
    {
        return size * num;
    }

    const bool followRedirect(true);

    const std::size_t defaultHttpTimeout(60 * 5);

    const std::map<char, std::string> sanitizers
    {
        { ' ', "%20" },
        { '!', "%21" },
        { '"', "%22" },
        { '#', "%23" },
        { '$', "%24" },
        { '\'', "%27" },
        { '(', "%28" },
        { ')', "%29" },
        { '*', "%2A" },
        { '+', "%2B" },
        { ',', "%2C" },
        { ';', "%3B" },
        { '<', "%3C" },
        { '>', "%3E" },
        { '@', "%40" },
        { '[', "%5B" },
        { '\\', "%5C" },
        { ']', "%5D" },
        { '^', "%5E" },
        { '`', "%60" },
        { '{', "%7B" },
        { '|', "%7C" },
        { '}', "%7D" },
        { '~', "%7E" }
    };
}

namespace arbiter
{
namespace drivers
{

Http::Http(HttpPool& pool) : m_pool(pool) { }

std::unique_ptr<Http> Http::create(HttpPool& pool, const Json::Value&)
{
    return std::unique_ptr<Http>(new Http(pool));
}

std::unique_ptr<std::size_t> Http::tryGetSize(std::string path) const
{
    std::unique_ptr<std::size_t> size;

    auto http(m_pool.acquire());
    HttpResponse res(http.head(path));

    if (res.ok())
    {
        if (res.headers().count("Content-Length"))
        {
            const std::string& str(res.headers().at("Content-Length"));
            size.reset(new std::size_t(std::stoul(str)));
        }
    }

    return size;
}

bool Http::get(std::string path, std::vector<char>& data) const
{
    bool good(false);

    auto http(m_pool.acquire());
    HttpResponse res(http.get(path));

    if (res.ok())
    {
        data = res.data();
        good = true;
    }

    return good;
}

void Http::put(std::string path, const std::vector<char>& data) const
{
    auto http(m_pool.acquire());

    if (!http.put(path, data).ok())
    {
        throw ArbiterError("Couldn't HTTP PUT to " + path);
    }
}

std::string Http::sanitize(std::string path)
{
    std::string result;

    for (const auto c : path)
    {
        auto it(sanitizers.find(c));

        if (it == sanitizers.end())
        {
            result += c;
        }
        else
        {
            result += it->second;
        }
    }

    return result;
}

} // namespace drivers

Curl::Curl(bool verbose, std::size_t timeout)
    : m_curl(0)
    , m_headers(0)
    , m_verbose(verbose)
    , m_timeout(timeout)
    , m_data()
{
    m_curl = curl_easy_init();
}

Curl::~Curl()
{
    curl_easy_cleanup(m_curl);
    curl_slist_free_all(m_headers);
    m_headers = 0;
}

void Curl::init(std::string path, const Headers& headers)
{
    // Reset our curl instance and header list.
    curl_slist_free_all(m_headers);
    m_headers = 0;

    // Set path.
    curl_easy_setopt(m_curl, CURLOPT_URL, path.c_str());

    // Needed for multithreaded Curl usage.
    curl_easy_setopt(m_curl, CURLOPT_NOSIGNAL, 1L);

    // Substantially faster DNS lookups without IPv6.
    curl_easy_setopt(m_curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);

    // Don't wait forever.
    curl_easy_setopt(m_curl, CURLOPT_TIMEOUT, m_timeout);

    // Configuration options.
    if (followRedirect) curl_easy_setopt(m_curl, CURLOPT_FOLLOWLOCATION, 1L);

    // Insert supplied headers.
    for (const auto& h : headers)
    {
        m_headers = curl_slist_append(
                m_headers,
                (h.first + ": " + h.second).c_str());
    }
}

HttpResponse Curl::get(std::string path, Headers headers)
{
    int httpCode(0);
    std::vector<char> data;

    if (m_verbose) curl_easy_setopt(m_curl, CURLOPT_VERBOSE, 1L);

    path = drivers::Http::sanitize(path);
    init(path, headers);

    // Register callback function and date pointer to consume the result.
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, getCb);
    curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &data);

    // Insert all headers into the request.
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, m_headers);

    // Set up callback and data pointer for received headers.
    Headers receivedHeaders;
    curl_easy_setopt(m_curl, CURLOPT_HEADERFUNCTION, headerCb);
    curl_easy_setopt(m_curl, CURLOPT_HEADERDATA, &receivedHeaders);

    // Run the command.
    curl_easy_perform(m_curl);
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_easy_reset(m_curl);
    return HttpResponse(httpCode, data, receivedHeaders);
}

HttpResponse Curl::head(std::string path, Headers headers)
{
    int httpCode(0);
    std::vector<char> data;

    if (m_verbose) curl_easy_setopt(m_curl, CURLOPT_VERBOSE, 1L);

    path = drivers::Http::sanitize(path);
    init(path, headers);

    // Register callback function and date pointer to consume the result.
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, getCb);
    curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &data);

    // Insert all headers into the request.
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, m_headers);

    // Set up callback and data pointer for received headers.
    Headers receivedHeaders;
    curl_easy_setopt(m_curl, CURLOPT_HEADERFUNCTION, headerCb);
    curl_easy_setopt(m_curl, CURLOPT_HEADERDATA, &receivedHeaders);

    // Specify a HEAD request.
    curl_easy_setopt(m_curl, CURLOPT_NOBODY, 1L);

    // Run the command.
    curl_easy_perform(m_curl);
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_easy_reset(m_curl);
    return HttpResponse(httpCode, data, receivedHeaders);
}

HttpResponse Curl::put(
        std::string path,
        const std::vector<char>& data,
        Headers headers)
{
    path = drivers::Http::sanitize(path);
    init(path, headers);

    if (m_verbose) curl_easy_setopt(m_curl, CURLOPT_VERBOSE, 1L);

    int httpCode(0);

    std::unique_ptr<PutData> putData(new PutData(data));

    // Register callback function and data pointer to create the request.
    curl_easy_setopt(m_curl, CURLOPT_READFUNCTION, putCb);
    curl_easy_setopt(m_curl, CURLOPT_READDATA, putData.get());

    // Insert all headers into the request.
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, m_headers);

    // Specify that this is a PUT request.
    curl_easy_setopt(m_curl, CURLOPT_PUT, 1L);

    // Must use this for binary data, otherwise curl will use strlen(), which
    // will likely be incorrect.
    curl_easy_setopt(
            m_curl,
            CURLOPT_INFILESIZE_LARGE,
            static_cast<curl_off_t>(data.size()));

    // Hide Curl's habit of printing things to console even with verbose set
    // to false.
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, eatLogging);

    // Run the command.
    curl_easy_perform(m_curl);
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_easy_reset(m_curl);
    return HttpResponse(httpCode);
}

HttpResponse Curl::post(
        std::string path,
        const std::vector<char>& data,
        Headers headers)
{
    path = drivers::Http::sanitize(path);
    init(path, headers);
    if (m_verbose) curl_easy_setopt(m_curl, CURLOPT_VERBOSE, 1L);

    int httpCode(0);

    std::unique_ptr<PutData> putData(new PutData(data));
    std::vector<char> writeData;

    // Register callback function and data pointer to create the request.
    curl_easy_setopt(m_curl, CURLOPT_READFUNCTION, putCb);
    curl_easy_setopt(m_curl, CURLOPT_READDATA, putData.get());

    // Register callback function and data pointer to consume the result.
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, getCb);
    curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &writeData);

    // Insert all headers into the request.
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, m_headers);

    // Set up callback and data pointer for received headers.
    Headers receivedHeaders;
    curl_easy_setopt(m_curl, CURLOPT_HEADERFUNCTION, headerCb);
    curl_easy_setopt(m_curl, CURLOPT_HEADERDATA, &receivedHeaders);

    // Specify that this is a POST request.
    curl_easy_setopt(m_curl, CURLOPT_POST, 1L);

    // Must use this for binary data, otherwise curl will use strlen(), which
    // will likely be incorrect.
    curl_easy_setopt(
            m_curl,
            CURLOPT_INFILESIZE_LARGE,
            static_cast<curl_off_t>(data.size()));

    // Run the command.
    curl_easy_perform(m_curl);
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_easy_reset(m_curl);
    HttpResponse response(httpCode, writeData, receivedHeaders);
    return response;
}

///////////////////////////////////////////////////////////////////////////////

HttpResource::HttpResource(
        HttpPool& pool,
        Curl& curl,
        const std::size_t id,
        const std::size_t retry)
    : m_pool(pool)
    , m_curl(curl)
    , m_id(id)
    , m_retry(retry)
{ }

HttpResource::~HttpResource()
{
    m_pool.release(m_id);
}

HttpResponse HttpResource::get(const std::string path, const Headers headers)
{
    auto f([this, path, headers]()->HttpResponse
    {
        return m_curl.get(path, headers);
    });

    return exec(f);
}

HttpResponse HttpResource::head( const std::string path, const Headers headers)
{
    auto f([this, path, headers]()->HttpResponse
    {
        return m_curl.head(path, headers);
    });

    return exec(f);
}

HttpResponse HttpResource::put(
        std::string path,
        const std::vector<char>& data,
        Headers headers)
{
    auto f([this, path, &data, headers]()->HttpResponse
    {
        return m_curl.put(path, data, headers);
    });

    return exec(f);
}

HttpResponse HttpResource::post(
        std::string path,
        const std::vector<char>& data,
        Headers headers)
{
    auto f([this, path, &data, headers]()->HttpResponse
    {
        return m_curl.post(path, data, headers);
    });

    return exec(f);
}

HttpResponse HttpResource::exec(std::function<HttpResponse()> f)
{
    HttpResponse res;
    std::size_t tries(0);

    do
    {
        res = f();
    }
    while (res.serverError() && tries++ < m_retry);

    return res;
}

///////////////////////////////////////////////////////////////////////////////

HttpPool::HttpPool(
        const std::size_t concurrent,
        const std::size_t retry,
        const Json::Value& json)
    : m_curls(concurrent)
    , m_available(concurrent)
    , m_retry(retry)
    , m_mutex()
    , m_cv()
{
    const bool verbose(
            json.isMember("arbiter") ?
                json["arbiter"]["verbose"].asBool() : false);

    const std::size_t timeout(
            json.isMember("http") && json["http"]["timeout"].asUInt64() ?
                json["http"]["timeout"].asUInt64() : defaultHttpTimeout);

    for (std::size_t i(0); i < concurrent; ++i)
    {
        m_available[i] = i;
        m_curls[i].reset(new Curl(verbose, timeout));
    }
}

HttpResource HttpPool::acquire()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this]()->bool { return !m_available.empty(); });

    const std::size_t id(m_available.back());
    Curl& curl(*m_curls[id]);

    m_available.pop_back();

    return HttpResource(*this, curl, id, m_retry);
}

void HttpPool::release(const std::size_t id)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_available.push_back(id);
    lock.unlock();

    m_cv.notify_one();
}

} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/http.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/s3.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/s3.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <thread>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/third/xml/xml.hpp>
#include <arbiter/util/crypto.hpp>
#endif

namespace arbiter
{

namespace
{
    const std::string baseUrl(".s3.amazonaws.com/");

    std::string getQueryString(const Query& query)
    {
        std::string result;

        bool first(true);
        for (const auto& q : query)
        {
            result += (first ? "?" : "&") + q.first + "=" + q.second;
            first = false;
        }

        return result;
    }

    struct Resource
    {
        Resource(std::string fullPath)
        {
            const std::size_t split(fullPath.find("/"));

            bucket = fullPath.substr(0, split);

            if (split != std::string::npos)
            {
                object = fullPath.substr(split + 1);
            }
        }

        std::string buildPath(Query query = Query()) const
        {
            const std::string queryString(getQueryString(query));
            return "http://" + bucket + baseUrl + object + queryString;
        }

        std::string bucket;
        std::string object;
    };

    typedef Xml::xml_node<> XmlNode;

    const std::string badResponse("Unexpected contents in AWS response");
}

namespace drivers
{

AwsAuth::AwsAuth(const std::string access, const std::string hidden)
    : m_access(access)
    , m_hidden(hidden)
{ }

std::unique_ptr<AwsAuth> AwsAuth::find(std::string user)
{
    std::unique_ptr<AwsAuth> auth;

    if (user.empty())
    {
        user = getenv("AWS_PROFILE") ? getenv("AWS_PROFILE") : "default";
    }

    drivers::Fs fs;
    std::unique_ptr<std::string> file(fs.tryGet("~/.aws/credentials"));

    // First, try reading credentials file.
    if (file)
    {
        std::size_t index(0);
        std::size_t pos(0);
        std::vector<std::string> lines;

        do
        {
            index = file->find('\n', pos);
            std::string line(file->substr(pos, index - pos));

            line.erase(
                    std::remove_if(line.begin(), line.end(), ::isspace),
                    line.end());

            lines.push_back(line);

            pos = index + 1;
        }
        while (index != std::string::npos);

        if (lines.size() >= 3)
        {
            std::size_t i(0);

            const std::string userFind("[" + user + "]");
            const std::string accessFind("aws_access_key_id=");
            const std::string hiddenFind("aws_secret_access_key=");

            while (i < lines.size() - 2 && !auth)
            {
                if (lines[i].find(userFind) != std::string::npos)
                {
                    const std::string& accessLine(lines[i + 1]);
                    const std::string& hiddenLine(lines[i + 2]);

                    std::size_t accessPos(accessLine.find(accessFind));
                    std::size_t hiddenPos(hiddenLine.find(hiddenFind));

                    if (
                            accessPos != std::string::npos &&
                            hiddenPos != std::string::npos)
                    {
                        const std::string access(
                                accessLine.substr(
                                    accessPos + accessFind.size(),
                                    accessLine.find(';')));

                        const std::string hidden(
                                hiddenLine.substr(
                                    hiddenPos + hiddenFind.size(),
                                    hiddenLine.find(';')));

                        auth.reset(new AwsAuth(access, hidden));
                    }
                }

                ++i;
            }
        }
    }

    // Fall back to environment settings.
    if (!auth)
    {
        if (getenv("AWS_ACCESS_KEY_ID") && getenv("AWS_SECRET_ACCESS_KEY"))
        {
            auth.reset(
                    new AwsAuth(
                        getenv("AWS_ACCESS_KEY_ID"),
                        getenv("AWS_SECRET_ACCESS_KEY")));
        }
        else if (
                getenv("AMAZON_ACCESS_KEY_ID") &&
                getenv("AMAZON_SECRET_ACCESS_KEY"))
        {
            auth.reset(
                    new AwsAuth(
                        getenv("AMAZON_ACCESS_KEY_ID"),
                        getenv("AMAZON_SECRET_ACCESS_KEY")));
        }
    }

    return auth;
}

std::string AwsAuth::access() const
{
    return m_access;
}

std::string AwsAuth::hidden() const
{
    return m_hidden;
}

S3::S3(HttpPool& pool, const AwsAuth auth)
    : m_pool(pool)
    , m_auth(auth)
{ }

std::unique_ptr<S3> S3::create(HttpPool& pool, const Json::Value& json)
{
    std::unique_ptr<S3> s3;

    if (!json.isNull() && json.isMember("access") & json.isMember("hidden"))
    {
        AwsAuth auth(json["access"].asString(), json["hidden"].asString());
        s3.reset(new S3(pool, auth));
    }
    else
    {
        auto auth(AwsAuth::find(json.isNull() ? "" : json["user"].asString()));
        if (auth) s3.reset(new S3(pool, *auth));
    }

    return s3;
}

std::unique_ptr<std::size_t> S3::tryGetSize(std::string rawPath) const
{
    std::unique_ptr<std::size_t> size;

    rawPath = Http::sanitize(rawPath);
    const Resource resource(rawPath);

    const std::string path(resource.buildPath());

    Headers headers(httpGetHeaders(rawPath, "HEAD"));

    auto http(m_pool.acquire());

    HttpResponse res(http.head(path, headers));

    if (res.ok())
    {
        if (res.headers().count("Content-Length"))
        {
            const std::string& str(res.headers().at("Content-Length"));
            size.reset(new std::size_t(std::stoul(str)));
        }
    }

    return size;
}

bool S3::get(std::string rawPath, std::vector<char>& data) const
{
    return buildRequestAndGet(rawPath, Query(), data);
}

bool S3::buildRequestAndGet(
        std::string rawPath,
        const Query& query,
        std::vector<char>& data,
        const Headers userHeaders) const
{
    rawPath = Http::sanitize(rawPath);
    const Resource resource(rawPath);

    const std::string path(resource.buildPath(query));

    Headers headers(httpGetHeaders(rawPath));
    for (const auto& h : userHeaders) headers[h.first] = h.second;

    auto http(m_pool.acquire());

    HttpResponse res(http.get(path, headers));

    if (res.ok())
    {
        data = res.data();
        return true;
    }
    else
    {
        return false;
    }
}

void S3::put(std::string rawPath, const std::vector<char>& data) const
{
    const Resource resource(rawPath);

    const std::string path(resource.buildPath());
    const Headers headers(httpPutHeaders(rawPath));

    auto http(m_pool.acquire());

    if (!http.put(path, data, headers).ok())
    {
        throw ArbiterError("Couldn't S3 PUT to " + rawPath);
    }
}

std::vector<std::string> S3::glob(std::string path, bool verbose) const
{
    std::vector<std::string> results;
    path.pop_back();

    const bool recursive(path.back() == '*');
    if (recursive) path.pop_back();

    // https://docs.aws.amazon.com/AmazonS3/latest/API/RESTBucketGET.html
    const Resource resource(path);
    const std::string& bucket(resource.bucket);
    const std::string& object(resource.object);
    const std::string prefix(resource.object.empty() ? "" : resource.object);

    Query query;

    if (prefix.size()) query["prefix"] = prefix;

    bool more(false);
    std::vector<char> data;

    do
    {
        if (verbose) std::cout << "." << std::flush;

        if (!buildRequestAndGet(resource.bucket + "/", query, data))
        {
            throw ArbiterError("Couldn't S3 GET " + resource.bucket);
        }

        data.push_back('\0');

        Xml::xml_document<> xml;

        try
        {
            xml.parse<0>(data.data());
        }
        catch (Xml::parse_error)
        {
            throw ArbiterError("Could not parse S3 response.");
        }

        if (XmlNode* topNode = xml.first_node("ListBucketResult"))
        {
            if (XmlNode* truncNode = topNode->first_node("IsTruncated"))
            {
                std::string t(truncNode->value());
                std::transform(t.begin(), t.end(), t.begin(), tolower);

                more = (t == "true");
            }

            if (XmlNode* conNode = topNode->first_node("Contents"))
            {
                for ( ; conNode; conNode = conNode->next_sibling())
                {
                    if (XmlNode* keyNode = conNode->first_node("Key"))
                    {
                        std::string key(keyNode->value());
                        const bool isSubdir(
                                key.find('/', prefix.size()) !=
                                std::string::npos);

                        // The prefix may contain slashes (i.e. is a sub-dir)
                        // but we only want to traverse into subdirectories
                        // beyond the prefix if recursive is true.
                        if ( recursive || !isSubdir)
                        {
                            results.push_back("s3://" + bucket + "/" + key);
                        }

                        if (more)
                        {
                            query["marker"] =
                                object + key.substr(prefix.size());
                        }
                    }
                    else
                    {
                        throw ArbiterError(badResponse);
                    }
                }
            }
            else
            {
                throw ArbiterError(badResponse);
            }
        }
        else
        {
            throw ArbiterError(badResponse);
        }

        xml.clear();
    }
    while (more);

    return results;
}

Headers S3::httpGetHeaders(std::string filePath, std::string request) const
{
    const std::string httpDate(getHttpDate());
    const std::string signedEncoded(
            getSignedEncodedString(request, filePath, httpDate));

    Headers headers;

    headers["Date"] = httpDate;
    headers["Authorization"] = "AWS " + m_auth.access() + ":" + signedEncoded;

    return headers;
}

Headers S3::httpPutHeaders(std::string filePath) const
{
    const std::string httpDate(getHttpDate());
    const std::string signedEncoded(
            getSignedEncodedString(
                "PUT",
                filePath,
                httpDate,
                "application/octet-stream"));

    Headers headers;

    headers["Content-Type"] = "application/octet-stream";
    headers["Date"] = httpDate;
    headers["Authorization"] = "AWS " + m_auth.access() + ":" + signedEncoded;
    headers["Transfer-Encoding"] = "";
    headers["Expect"] = "";

    return headers;
}

std::string S3::getHttpDate() const
{
    time_t rawTime;
    char charBuf[80];

    time(&rawTime);

#ifndef ARBITER_WINDOWS
    tm* timeInfoPtr = localtime(&rawTime);
#else
    tm timeInfo;
    localtime_s(&timeInfo, &rawTime);
    tm* timeInfoPtr(&timeInfo);
#endif

    strftime(charBuf, 80, "%a, %d %b %Y %H:%M:%S %z", timeInfoPtr);
    std::string stringBuf(charBuf);

    return stringBuf;
}

std::string S3::getSignedEncodedString(
        std::string command,
        std::string file,
        std::string httpDate,
        std::string contentType) const
{
    const std::string toSign(getStringToSign(
                command,
                file,
                httpDate,
                contentType));

    const std::vector<char> signedData(signString(toSign));
    return crypto::encodeBase64(signedData);
}

std::string S3::getStringToSign(
        std::string command,
        std::string file,
        std::string httpDate,
        std::string contentType) const
{
    return
        command + "\n" +
        "\n" +
        contentType + "\n" +
        httpDate + "\n" +
        "/" + file;
}

std::vector<char> S3::signString(std::string input) const
{
    return crypto::hmacSha1(m_auth.hidden(), input);
}



// These functions allow a caller to directly pass additional headers into
// their GET request.  This is only applicable when using the S3 driver
// directly, as these are not available through the Arbiter.

std::vector<char> S3::getBinary(std::string rawPath, Headers headers) const
{
    std::vector<char> data;
    const std::string stripped(Arbiter::stripType(rawPath));

    if (!buildRequestAndGet(stripped, Query(), data, headers))
    {
        throw ArbiterError("Couldn't S3 GET " + rawPath);
    }

    return data;
}

std::string S3::get(std::string rawPath, Headers headers) const
{
    std::vector<char> data(getBinary(rawPath, headers));
    return std::string(data.begin(), data.end());
}

} // namespace drivers
} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/s3.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/dropbox.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <functional>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/drivers/dropbox.hpp>
#include <arbiter/third/xml/xml.hpp>
#include <arbiter/util/crypto.hpp>

#ifndef ARBITER_EXTERNAL_JSON
#include <arbiter/third/json/json.hpp>
#endif

#endif



#ifdef ARBITER_EXTERNAL_JSON
#include <json/json.h>
#endif

namespace arbiter
{

namespace
{
    const std::string baseGetUrl("https://content.dropboxapi.com/");
    const std::string getUrlV1(baseGetUrl + "1/files/auto/");
    const std::string getUrlV2(baseGetUrl + "2/files/download");

    // We still need to use API V1 for GET requests since V2 is poorly
    // documented and doesn't correctly support the Range header.  Hopefully
    // we can switch to V2 at some point.
    const bool legacy(true);

    const std::string listUrl("https://api.dropboxapi.com/2/files/list_folder");
    const std::string metaUrl("https://api.dropboxapi.com/2/files/get_metadata");
    const std::string continueListUrl(listUrl + "/continue");

    const auto ins([](unsigned char lhs, unsigned char rhs)
    {
        return std::tolower(lhs) == std::tolower(rhs);
    });

    std::string toSanitizedString(const Json::Value& v)
    {
        Json::FastWriter writer;
        std::string f(writer.write(v));
        f.erase(std::remove(f.begin(), f.end(), '\n'), f.end());
        return f;
    }

    const std::string dirTag("folder");
    const std::string fileTag("file");
}

namespace drivers
{

Dropbox::Dropbox(HttpPool& pool, const DropboxAuth auth)
    : m_pool(pool)
    , m_auth(auth)
{ }

std::unique_ptr<Dropbox> Dropbox::create(
        HttpPool& pool,
        const Json::Value& json)
{
    std::unique_ptr<Dropbox> dropbox;

    if (!json.isNull() && json.isMember("token"))
    {
        dropbox.reset(new Dropbox(pool, DropboxAuth(json["token"].asString())));
    }

    return dropbox;
}

Headers Dropbox::httpGetHeaders() const
{
    Headers headers;

    headers["Authorization"] = "Bearer " + m_auth.token();

    if (!legacy)
    {
        headers["Transfer-Encoding"] = "";
        headers["Expect"] = "";
    }

    return headers;
}

Headers Dropbox::httpPostHeaders() const
{
    Headers headers;

    headers["Authorization"] = "Bearer " + m_auth.token();
    headers["Transfer-Encoding"] = "chunked";
    headers["Expect"] = "100-continue";
    headers["Content-Type"] = "application/json";

    return headers;
}

bool Dropbox::get(const std::string rawPath, std::vector<char>& data) const
{
    return buildRequestAndGet(rawPath, data);
}

std::unique_ptr<std::size_t> Dropbox::tryGetSize(
        const std::string rawPath) const
{
    std::unique_ptr<std::size_t> result;

    Headers headers(httpPostHeaders());

    Json::Value json;
    json["path"] = std::string("/" + Http::sanitize(rawPath));
    const auto f(toSanitizedString(json));
    const std::vector<char> postData(f.begin(), f.end());

    auto http(m_pool.acquire());
    HttpResponse res(http.post(metaUrl, postData, headers));

    if (res.ok())
    {
        const auto data(res.data());

        Json::Value json;
        Json::Reader reader;
        reader.parse(std::string(data.data(), data.size()), json, false);

        if (json.isMember("size"))
        {
            result.reset(new std::size_t(json["size"].asUInt64()));
        }
    }

    return result;
}

bool Dropbox::buildRequestAndGet(
        const std::string rawPath,
        std::vector<char>& data,
        const Headers userHeaders) const
{
    const std::string path(Http::sanitize(rawPath));

    Headers headers(httpGetHeaders());

    if (!legacy)
    {
        Json::Value json;
        json["path"] = std::string("/" + path);
        headers["Dropbox-API-Arg"] = toSanitizedString(json);
    }

    headers.insert(userHeaders.begin(), userHeaders.end());

    auto http(m_pool.acquire());

    HttpResponse res(
            legacy ?
                http.get(getUrlV1 + path, headers) :
                http.get(getUrlV2, headers));

    if (res.ok())
    {
        if (
                (legacy && !res.headers().count("Content-Length")) ||
                (!legacy && !res.headers().count("original-content-length")))
        {
            return false;
        }

        const std::size_t size(
                std::stol(
                    legacy ?
                        res.headers().at("Content-Length") :
                        res.headers().at("original-content-length")));

        data = res.data();

        if (size == res.data().size())
        {
            return true;
        }
        else
        {
            std::ostringstream oss;
            oss << "Data size check failed. State size was '" << size
                << "' and downloaded size was '" << res.data().size() << "'";
            throw ArbiterError(oss.str());
        }
    }
    else
    {
        std::string message(res.data().data(), res.data().size());
        throw ArbiterError(
                "Server response: " + std::to_string(res.code()) + " - '" +
                message + "'");
    }

    return false;
}

void Dropbox::put(std::string rawPath, const std::vector<char>& data) const
{
    throw ArbiterError("PUT not yet supported for " + type());
}

std::string Dropbox::continueFileInfo(std::string cursor) const
{
    Headers headers(httpPostHeaders());

    auto http(m_pool.acquire());

    Json::Value json;
    json["cursor"] = cursor;
    const std::string f(toSanitizedString(json));

    std::vector<char> postData(f.begin(), f.end());
    HttpResponse res(http.post(continueListUrl, postData, headers));

    if (res.ok())
    {
        return std::string(res.data().data(), res.data().size());
    }
    else
    {
        std::string message(res.data().data(), res.data().size());
        throw ArbiterError(
                "Server response: " + std::to_string(res.code()) + " - '" +
                message + "'");
    }

    return std::string("");
}

std::vector<std::string> Dropbox::glob(std::string rawPath, bool verbose) const
{
    std::vector<std::string> results;

    const std::string path(
            Http::sanitize(rawPath.substr(0, rawPath.size() - 2)));

    auto listPath = [this](std::string path)->std::string
    {
        auto http(m_pool.acquire());
        Headers headers(httpPostHeaders());

        Json::Value request;
        request["path"] = std::string("/" + path);
        request["recursive"] = false;
        request["include_media_info"] = false;
        request["include_deleted"] = false;

        std::string f = toSanitizedString(request);

        std::vector<char> postData(f.begin(), f.end());
        HttpResponse res(http.post(listUrl, postData, headers));

        if (res.ok())
        {
            return std::string(res.data().data(), res.data().size());
        }
        else if (res.code() == 409)
        {
            return "";
        }
        else
        {
            std::string message(res.data().data(), res.data().size());
            throw ArbiterError(
                    "Server response: " + std::to_string(res.code()) + " - '" +
                    message + "'");
        }
    };

    bool more(false);
    std::string cursor("");

    auto processPath = [verbose, &results, &more, &cursor](std::string data)
    {
        if (data.empty()) return;

        if (verbose) std::cout << '.';

        Json::Value json;
        Json::Reader reader;
        reader.parse(data, json, false);

        const Json::Value& entries(json["entries"]);

        if (entries.isNull())
        {
            throw ArbiterError("Returned JSON from Dropbox was NULL");
        }
        if (!entries.isArray())
        {
            throw ArbiterError("Returned JSON from Dropbox was not an array");
        }

        more = json["has_more"].asBool();
        cursor = json["cursor"].asString();

        for (std::size_t i(0); i < entries.size(); ++i)
        {
            const Json::Value& v(entries[static_cast<Json::ArrayIndex>(i)]);
            const std::string tag(v[".tag"].asString());

            // Only insert files.
            if (std::equal(tag.begin(), tag.end(), fileTag.begin(), ins))
            {
                // Results already begin with a slash.
                results.push_back("dropbox:/" + v["path_lower"].asString());
            }
        }
    };

    processPath(listPath(path));

    if (more)
    {
        do
        {
            processPath(continueFileInfo(cursor));
        }
        while (more);
    }

    return results;
}



// These functions allow a caller to directly pass additional headers into
// their GET request.  This is only applicable when using the Dropbox driver
// directly, as these are not available through the Arbiter.

std::vector<char> Dropbox::getBinary(std::string rawPath, Headers headers) const
{
    std::vector<char> data;
    const std::string stripped(Arbiter::stripType(rawPath));
    if (!buildRequestAndGet(stripped, data, headers))
    {
        throw ArbiterError("Couldn't Dropbox GET " + rawPath);
    }

    return data;
}

std::string Dropbox::get(std::string rawPath, Headers headers) const
{
    std::vector<char> data(getBinary(rawPath, headers));
    return std::string(data.begin(), data.end());
}

} // namespace drivers
} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/dropbox.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/crypto.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/crypto.hpp>
#endif

#include <cstdint>

#define ROTLEFT(a, b) ((a << b) | (a >> (32 - b)))

namespace arbiter
{
namespace crypto
{
namespace
{
    const std::string base64Vals(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/");

    const std::string hexVals("0123456789abcdef");

    const std::size_t block(64);

    std::vector<char> append(
            const std::vector<char>& a,
            const std::vector<char>& b)
    {
        std::vector<char> out(a);
        out.insert(out.end(), b.begin(), b.end());
        return out;
    }

    std::vector<char> append(
            const std::vector<char>& a,
            const std::string& b)
    {
        return append(a, std::vector<char>(b.begin(), b.end()));
    }

    // SHA1 implementation:
    //      https://github.com/B-Con/crypto-algorithms
    //
    // HMAC:
    //      https://en.wikipedia.org/wiki/Hash-based_message_authentication_code
    //
    typedef struct
    {
        uint8_t data[64];
        uint32_t datalen;
        unsigned long long bitlen;
        uint32_t state[5];
        uint32_t k[4];
    } SHA1_CTX;

    void sha1_transform(SHA1_CTX *ctx, const uint8_t* data)
    {
        uint32_t a, b, c, d, e, i, j, t, m[80];

        for (i = 0, j = 0; i < 16; ++i, j += 4)
        {
            m[i] =
                (data[j] << 24) + (data[j + 1] << 16) +
                (data[j + 2] << 8) + (data[j + 3]);
        }

        for ( ; i < 80; ++i)
        {
            m[i] = (m[i - 3] ^ m[i - 8] ^ m[i - 14] ^ m[i - 16]);
            m[i] = (m[i] << 1) | (m[i] >> 31);
        }

        a = ctx->state[0];
        b = ctx->state[1];
        c = ctx->state[2];
        d = ctx->state[3];
        e = ctx->state[4];

        for (i = 0; i < 20; ++i) {
            t = ROTLEFT(a, 5) + ((b & c) ^ (~b & d)) + e + ctx->k[0] + m[i];
            e = d;
            d = c;
            c = ROTLEFT(b, 30);
            b = a;
            a = t;
        }
        for ( ; i < 40; ++i) {
            t = ROTLEFT(a, 5) + (b ^ c ^ d) + e + ctx->k[1] + m[i];
            e = d;
            d = c;
            c = ROTLEFT(b, 30);
            b = a;
            a = t;
        }
        for ( ; i < 60; ++i) {
            t = ROTLEFT(a, 5) + ((b & c) ^ (b & d) ^ (c & d)) + e +
                ctx->k[2] + m[i];
            e = d;
            d = c;
            c = ROTLEFT(b, 30);
            b = a;
            a = t;
        }
        for ( ; i < 80; ++i) {
            t = ROTLEFT(a, 5) + (b ^ c ^ d) + e + ctx->k[3] + m[i];
            e = d;
            d = c;
            c = ROTLEFT(b, 30);
            b = a;
            a = t;
        }

        ctx->state[0] += a;
        ctx->state[1] += b;
        ctx->state[2] += c;
        ctx->state[3] += d;
        ctx->state[4] += e;
    }

    void sha1_init(SHA1_CTX *ctx)
    {
        ctx->datalen = 0;
        ctx->bitlen = 0;
        ctx->state[0] = 0x67452301;
        ctx->state[1] = 0xEFCDAB89;
        ctx->state[2] = 0x98BADCFE;
        ctx->state[3] = 0x10325476;
        ctx->state[4] = 0xc3d2e1f0;
        ctx->k[0] = 0x5a827999;
        ctx->k[1] = 0x6ed9eba1;
        ctx->k[2] = 0x8f1bbcdc;
        ctx->k[3] = 0xca62c1d6;
    }

    void sha1_update(SHA1_CTX *ctx, const uint8_t* data, size_t len)
    {
        for (std::size_t i(0); i < len; ++i)
        {
            ctx->data[ctx->datalen] = data[i];
            ++ctx->datalen;
            if (ctx->datalen == 64)
            {
                sha1_transform(ctx, ctx->data);
                ctx->bitlen += 512;
                ctx->datalen = 0;
            }
        }
    }

    void sha1_final(SHA1_CTX *ctx, uint8_t* hash)
    {
        uint32_t i;

        i = ctx->datalen;

        // Pad whatever data is left in the buffer.
        if (ctx->datalen < 56)
        {
            ctx->data[i++] = 0x80;
            while (i < 56)
                ctx->data[i++] = 0x00;
        }
        else
        {
            ctx->data[i++] = 0x80;
            while (i < 64)
                ctx->data[i++] = 0x00;
            sha1_transform(ctx, ctx->data);
            std::memset(ctx->data, 0, 56);
        }

        // Append to the padding the total message's length in bits and
        // transform.
        ctx->bitlen += ctx->datalen * 8;
        ctx->data[63] = static_cast<uint8_t>(ctx->bitlen);
        ctx->data[62] = static_cast<uint8_t>(ctx->bitlen >> 8);
        ctx->data[61] = static_cast<uint8_t>(ctx->bitlen >> 16);
        ctx->data[60] = static_cast<uint8_t>(ctx->bitlen >> 24);
        ctx->data[59] = static_cast<uint8_t>(ctx->bitlen >> 32);
        ctx->data[58] = static_cast<uint8_t>(ctx->bitlen >> 40);
        ctx->data[57] = static_cast<uint8_t>(ctx->bitlen >> 48);
        ctx->data[56] = static_cast<uint8_t>(ctx->bitlen >> 56);
        sha1_transform(ctx, ctx->data);

        // Since this implementation uses little endian byte ordering and MD
        // uses big endian, reverse all the bytes when copying the final state
        // to the output hash.
        for (i = 0; i < 4; ++i)
        {
            hash[i]      = (ctx->state[0] >> (24 - i * 8)) & 0x000000ff;
            hash[i + 4]  = (ctx->state[1] >> (24 - i * 8)) & 0x000000ff;
            hash[i + 8]  = (ctx->state[2] >> (24 - i * 8)) & 0x000000ff;
            hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0x000000ff;
            hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0x000000ff;
        }
    }

    std::vector<char> sha1(const std::vector<char>& data)
    {
        SHA1_CTX ctx;
        std::vector<char> out(20);

        sha1_init(&ctx);
        sha1_update(
                &ctx,
                reinterpret_cast<const uint8_t*>(data.data()),
                data.size());
        sha1_final(&ctx, reinterpret_cast<uint8_t*>(out.data()));

        return out;
    }

    std::string sha1(const std::string& data)
    {
        auto hashed(sha1(std::vector<char>(data.begin(), data.end())));
        return std::string(hashed.begin(), hashed.end());
    }

} // unnamed namespace

std::vector<char> hmacSha1(std::string key, const std::string message)
{
    if (key.size() > block) key = sha1(key);
    if (key.size() < block) key.insert(key.end(), block - key.size(), 0);

    std::vector<char> okeypad(block, 0x5c);
    std::vector<char> ikeypad(block, 0x36);

    for (std::size_t i(0); i < block; ++i)
    {
        okeypad[i] ^= key[i];
        ikeypad[i] ^= key[i];
    }

    return sha1(append(okeypad, sha1(append(ikeypad, message))));
}

std::string encodeBase64(const std::vector<char>& data)
{
    std::vector<uint8_t> input;
    for (std::size_t i(0); i < data.size(); ++i)
    {
        char c(data[i]);
        input.push_back(*reinterpret_cast<uint8_t*>(&c));
    }

    std::size_t fullSteps(input.size() / 3);
    while (input.size() % 3) input.push_back(0);
    uint8_t* pos(input.data());
    uint8_t* end(input.data() + fullSteps * 3);

    std::string output(fullSteps * 4, '_');
    std::size_t outIndex(0);

    const uint32_t mask(0x3F);

    while (pos != end)
    {
        uint32_t chunk((*pos) << 16 | *(pos + 1) << 8 | *(pos + 2));

        output[outIndex++] = base64Vals[(chunk >> 18) & mask];
        output[outIndex++] = base64Vals[(chunk >> 12) & mask];
        output[outIndex++] = base64Vals[(chunk >>  6) & mask];
        output[outIndex++] = base64Vals[chunk & mask];

        pos += 3;
    }

    if (end != input.data() + input.size())
    {
        const std::size_t num(pos - end == 1 ? 2 : 3);
        uint32_t chunk(*(pos) << 16 | *(pos + 1) << 8 | *(pos + 2));

        output.push_back(base64Vals[(chunk >> 18) & mask]);
        output.push_back(base64Vals[(chunk >> 12) & mask]);
        if (num == 3) output.push_back(base64Vals[(chunk >> 6) & mask]);
    }

    while (output.size() % 4) output.push_back('=');

    return output;
}

std::string encodeAsHex(const std::vector<char>& input)
{
    std::string output;
    output.reserve(input.size() * 2);

    uint8_t u(0);

    for (const char c : input)
    {
        u = *reinterpret_cast<const uint8_t*>(&c);
        output.push_back(hexVals[u >> 4]);
        output.push_back(hexVals[u & 0x0F]);
    }

    return output;
}

} // namespace crypto
} // namespace arbiter


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/crypto.cpp
// //////////////////////////////////////////////////////////////////////





