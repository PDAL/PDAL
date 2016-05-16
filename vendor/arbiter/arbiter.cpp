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
#include <arbiter/util/util.hpp>
#endif

#include <algorithm>
#include <cstdlib>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

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
        outEndpoint.putSubpath(util::getBasename(path), getBinary(path));
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

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
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

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
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

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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
#include <windows.h>
#endif

#include <cstdlib>
#include <fstream>
#include <stdexcept>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

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

    LPWIN32_FIND_DATAW data{};
    HANDLE hFind(FindFirstFileW(wide.c_str(), data));

    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if ((data->dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
            {
                results.push_back(converter.to_bytes(data->cFileName));
            }
        }
        while (FindNextFileW(hFind, data));
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
        char* userProfile(nullptr);
        char* homePath(nullptr);
        char* homeDrive(nullptr);

        std::size_t len(0);
        errno_t err(0);

        err = _dupenv_s(&userProfile, &len, "USERPROFILE");
        err = _dupenv_s(&homeDrive, &len, "HOMEDRIVE");
        err = _dupenv_s(&homePath, &len, "HOMEPATH");

        if (!userProfile && !(homeDrive && homePath))
        {
            noHome();
        }

        static const std::string home(
                userProfile ?
                    userProfile :
                    std::string(homeDrive) + std::string(homePath));
#endif

        out = home + in.substr(1);
    }

    return out;
}

std::string getTempPath()
{
#ifndef ARBITER_WINDOWS
    if (const char* t = getenv("TMPDIR"))   return t;
    if (const char* t = getenv("TMP"))      return t;
    if (const char* t = getenv("TEMP"))     return t;
    if (const char* t = getenv("TEMPDIR"))  return t;
    return "/tmp";
#else
    std::vector<char> path(MAX_PATH, '\0');
    if (GetTempPath(MAX_PATH, path.data())) return path.data();
    else throw ArbiterError("Could not find a temp path.");
#endif
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

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

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
        { '/', "%2F" },
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
} // unnamed namespace

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

std::string Http::sanitize(const std::string path, const std::string exclusions)
{
    std::string result;

    for (const auto c : path)
    {
        auto it(sanitizers.find(c));

        if (it == sanitizers.end() || exclusions.find(c) != std::string::npos)
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

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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
#include <ctime>
#include <functional>
#include <iostream>
#include <numeric>
#include <thread>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/third/xml/xml.hpp>
#include <arbiter/util/md5.hpp>
#include <arbiter/util/transforms.hpp>
#include <arbiter/util/sha256.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace
{
    const std::string dateFormat("%Y%m%d");
    const std::string timeFormat("%H%M%S");

    std::string getBaseUrl(const std::string& region)
    {
        // https://docs.aws.amazon.com/general/latest/gr/rande.html#s3_region
        if (region == "us-east-1") return ".s3.amazonaws.com/";
        else return ".s3-" + region + ".amazonaws.com/";
    }

    drivers::Fs fsDriver;

    std::string line(const std::string& data) { return data + "\n"; }
    const std::vector<char> empty;

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

    typedef Xml::xml_node<> XmlNode;
    const std::string badResponse("Unexpected contents in AWS response");

    std::string toLower(const std::string& in)
    {
        return std::accumulate(
                in.begin(),
                in.end(),
                std::string(),
                [](const std::string& out, const char c)
                {
                    return out + static_cast<char>(::tolower(c));
                });
    }

    // Trims sequential whitespace into a single character, and trims all
    // leading and trailing whitespace.
    std::string trim(const std::string& in)
    {
        std::string s = std::accumulate(
                in.begin(),
                in.end(),
                std::string(),
                [](const std::string& out, const char c)
                {
                    if (
                        std::isspace(c) &&
                        (out.empty() || std::isspace(out.back())))
                    {
                        return out;
                    }
                    else
                    {
                        return out + c;
                    }
                });

        // Might have one trailing whitespace character.
        if (s.size() && std::isspace(s.back())) s.pop_back();
        return s;
    }

    std::vector<std::string> condense(const std::vector<std::string>& in)
    {
        return std::accumulate(
                in.begin(),
                in.end(),
                std::vector<std::string>(),
                [](const std::vector<std::string>& base, const std::string& in)
                {
                    auto out(base);

                    std::string current(in);
                    current.erase(
                            std::remove_if(
                                current.begin(),
                                current.end(),
                                [](char c) { return std::isspace(c); }),
                            current.end());

                    out.push_back(current);
                    return out;
                });
    }

    std::vector<std::string> split(const std::string& in, char delimiter = '\n')
    {
        std::size_t index(0);
        std::size_t pos(0);
        std::vector<std::string> lines;

        do
        {
            index = in.find(delimiter, pos);
            std::string line(in.substr(pos, index - pos));

            line.erase(
                    std::remove_if(line.begin(), line.end(), ::isspace),
                    line.end());

            lines.push_back(line);

            pos = index + 1;
        }
        while (index != std::string::npos);

        return lines;
    }
}

namespace drivers
{

AwsAuth::AwsAuth(const std::string access, const std::string hidden)
    : m_access(access)
    , m_hidden(hidden)
{ }

std::unique_ptr<AwsAuth> AwsAuth::find(std::string profile)
{
    std::unique_ptr<AwsAuth> auth;

    const std::string credFile("~/.aws/credentials");

    // First, try reading credentials file.
    if (std::unique_ptr<std::string> cred = fsDriver.tryGet(credFile))
    {
        const std::vector<std::string> lines(condense(split(*cred)));

        if (lines.size() >= 3)
        {
            std::size_t i(0);

            const std::string profileFind("[" + profile + "]");
            const std::string accessFind("aws_access_key_id=");
            const std::string hiddenFind("aws_secret_access_key=");

            while (i < lines.size() - 2 && !auth)
            {
                if (lines[i].find(profileFind) != std::string::npos)
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

S3::S3(
        HttpPool& pool,
        const AwsAuth auth,
        const std::string region,
        const bool sse)
    : m_pool(pool)
    , m_auth(auth)
    , m_region(region)
    , m_baseUrl(getBaseUrl(region))
    , m_baseHeaders()
{
    if (sse)
    {
        // This could grow to support other SSE schemes, like KMS and customer-
        // supplied keys.
        m_baseHeaders["x-amz-server-side-encryption"] = "AES256";
    }
}

std::unique_ptr<S3> S3::create(HttpPool& pool, const Json::Value& json)
{
    std::unique_ptr<AwsAuth> auth;
    std::unique_ptr<S3> s3;

    const std::string profile(extractProfile(json));
    const bool sse(json["sse"].asBool());

    if (!json.isNull() && json.isMember("access") & json.isMember("hidden"))
    {
        auth.reset(
                new AwsAuth(
                    json["access"].asString(),
                    json["hidden"].asString()));
    }
    else
    {
        auth = AwsAuth::find(profile);
    }

    if (!auth) return s3;

    // Try to get the region from the config file, or default to US standard.
    std::string region("us-east-1");

    if (std::unique_ptr<std::string> config = fsDriver.tryGet("~/.aws/config"))
    {
        const std::vector<std::string> lines(condense(split(*config)));
        bool regionFound(false);

        if (lines.size() >= 3)
        {
            std::size_t i(0);

            const std::string profileFind("[" + profile + "]");
            const std::string outputFind("output=");
            const std::string regionFind("region=");

            while (i < lines.size() - 2 && !regionFound)
            {
                if (lines[i].find(profileFind) != std::string::npos)
                {
                    const std::string& outputLine(lines[i + 1]);
                    const std::string& regionLine(lines[i + 2]);

                    std::size_t outputPos(outputLine.find(outputFind));
                    std::size_t regionPos(regionLine.find(regionFind));

                    if (
                            outputPos != std::string::npos &&
                            regionPos != std::string::npos)
                    {
                        region = regionLine.substr(
                                regionPos + regionFind.size(),
                                regionLine.find(';'));

                        regionFound = true;
                    }
                }

                ++i;
            }
        }
    }

    s3.reset(new S3(pool, *auth, region, sse));

    return s3;
}

std::string S3::extractProfile(const Json::Value& json)
{
    if (
            !json.isNull() &&
            json.isMember("profile") &&
            json["profile"].asString().size())
    {
        return json["profile"].asString();
    }
    else
    {
        return getenv("AWS_PROFILE") ? getenv("AWS_PROFILE") : "default";
    }
}

std::unique_ptr<std::size_t> S3::tryGetSize(std::string rawPath) const
{
    std::unique_ptr<std::size_t> size;

    const Resource resource(m_baseUrl, rawPath);
    const AuthV4 authV4(
            "HEAD",
            m_region,
            resource,
            m_auth,
            Query(),
            Headers(),
            empty);

    auto http(m_pool.acquire());
    HttpResponse res(http.head(resource.buildPath(), authV4.headers()));

    if (res.ok() && res.headers().count("Content-Length"))
    {
        const std::string& str(res.headers().at("Content-Length"));
        size.reset(new std::size_t(std::stoul(str)));
    }

    return size;
}

bool S3::get(std::string rawPath, std::vector<char>& data) const
{
    return get(rawPath, Query(), Headers(), data);
}

bool S3::get(
        std::string rawPath,
        const Query& query,
        const Headers& headers,
        std::vector<char>& data) const
{
    const Resource resource(m_baseUrl, rawPath);
    const AuthV4 authV4(
            "GET",
            m_region,
            resource,
            m_auth,
            query,
            headers,
            empty);

    auto http(m_pool.acquire());
    HttpResponse res(http.get(resource.buildPath(query), authV4.headers()));

    if (res.ok())
    {
        data = res.data();
        return true;
    }
    else
    {
        std::cout << std::string(res.data().data(), res.data().size()) <<
            std::endl;
        return false;
    }
}

void S3::put(std::string rawPath, const std::vector<char>& data) const
{
    const Resource resource(m_baseUrl, rawPath);

    Headers headers(m_baseHeaders);
    const AuthV4 authV4(
            "PUT",
            m_region,
            resource,
            m_auth,
            Query(),
            headers,
            data);

    auto http(m_pool.acquire());
    HttpResponse res(http.put(resource.buildPath(), data, authV4.headers()));

    if (!res.ok())
    {
        throw ArbiterError(
                "Couldn't S3 PUT to " + rawPath + ": " +
                std::string(res.data().data(), res.data().size()));
    }
}

std::vector<std::string> S3::glob(std::string path, bool verbose) const
{
    std::vector<std::string> results;
    path.pop_back();

    const bool recursive(path.back() == '*');
    if (recursive) path.pop_back();

    // https://docs.aws.amazon.com/AmazonS3/latest/API/RESTBucketGET.html
    const Resource resource(m_baseUrl, path);
    const std::string& bucket(resource.bucket);
    const std::string& object(resource.object);

    Query query;

    if (object.size()) query["prefix"] = object;

    bool more(false);
    std::vector<char> data;

    do
    {
        if (verbose) std::cout << "." << std::flush;

        if (!get(resource.bucket + "/", query, Headers(), data))
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
                std::transform(t.begin(), t.end(), t.begin(), ::tolower);

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
                                key.find('/', object.size()) !=
                                std::string::npos);

                        // The prefix may contain slashes (i.e. is a sub-dir)
                        // but we only want to traverse into subdirectories
                        // beyond the prefix if recursive is true.
                        if (recursive || !isSubdir)
                        {
                            results.push_back("s3://" + bucket + "/" + key);
                        }

                        if (more)
                        {
                            query["marker"] =
                                object + key.substr(object.size());
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

S3::AuthV4::AuthV4(
        const std::string verb,
        const std::string& region,
        const Resource& resource,
        const AwsAuth& auth,
        const Query& query,
        const Headers& headers,
        const std::vector<char>& data)
    : m_auth(auth)
    , m_region(region)
    , m_formattedTime()
    , m_headers(headers)
    , m_signedHeadersString()
{
    m_headers["Host"] = resource.host();
    m_headers["X-Amz-Date"] = m_formattedTime.amazonDate();
    m_headers["X-Amz-Content-Sha256"] =
            crypto::encodeAsHex(crypto::sha256(data));

    if (verb == "PUT" || verb == "POST")
    {
        m_headers["Content-Type"] = "application/octet-stream";
        m_headers["Transfer-Encoding"] = "";
        m_headers["Expect"] = "";
    }

    const Headers normalizedHeaders(
            std::accumulate(
                m_headers.begin(),
                m_headers.end(),
                Headers(),
                [](const Headers& in, const Headers::value_type& h)
                {
                    Headers out(in);
                    out[toLower(h.first)] = trim(h.second);
                    return out;
                }));

    m_canonicalHeadersString =
            std::accumulate(
                normalizedHeaders.begin(),
                normalizedHeaders.end(),
                std::string(),
                [](const std::string& in, const Headers::value_type& h)
                {
                    return in + h.first + ':' + h.second + '\n';
                });

    m_signedHeadersString =
            std::accumulate(
                normalizedHeaders.begin(),
                normalizedHeaders.end(),
                std::string(),
                [](const std::string& in, const Headers::value_type& h)
                {
                    return in + (in.empty() ? "" : ";") + h.first;
                });

    const std::string canonicalRequest(
            buildCanonicalRequest(verb, resource, query, data));

    const std::string stringToSign(buildStringToSign(canonicalRequest));

    const std::string signature(calculateSignature(stringToSign));

    m_headers["Authorization"] =
            getAuthHeader(m_signedHeadersString, signature);
}

std::string S3::AuthV4::buildCanonicalRequest(
        const std::string verb,
        const Resource& resource,
        const Query& query,
        const std::vector<char>& data) const
{
    const std::string canonicalUri(Http::sanitize("/" + resource.object));

    auto canonicalizeQuery([](const std::string& s, const Query::value_type& q)
    {
        const std::string keyVal(
                Http::sanitize(q.first, "") + '=' +
                Http::sanitize(q.second, ""));

        return s + (s.size() ? "&" : "") + keyVal;
    });

    const std::string canonicalQuery(
            std::accumulate(
                query.begin(),
                query.end(),
                std::string(),
                canonicalizeQuery));

    return
        line(verb) +
        line(canonicalUri) +
        line(canonicalQuery) +
        line(m_canonicalHeadersString) +
        line(m_signedHeadersString) +
        crypto::encodeAsHex(crypto::sha256(data));
}

std::string S3::AuthV4::buildStringToSign(
        const std::string& canonicalRequest) const
{
    return
        line("AWS4-HMAC-SHA256") +
        line(m_formattedTime.amazonDate()) +
        line(m_formattedTime.date() + "/" + m_region + "/s3/aws4_request") +
        crypto::encodeAsHex(crypto::sha256(canonicalRequest));
}

std::string S3::AuthV4::calculateSignature(
        const std::string& stringToSign) const
{
    const std::string kDate(
            crypto::hmacSha256(
                "AWS4" + m_auth.hidden(),
                m_formattedTime.date()));

    const std::string kRegion(crypto::hmacSha256(kDate, m_region));
    const std::string kService(crypto::hmacSha256(kRegion, "s3"));
    const std::string kSigning(
            crypto::hmacSha256(kService, "aws4_request"));

    return crypto::encodeAsHex(crypto::hmacSha256(kSigning, stringToSign));
}

std::string S3::AuthV4::getAuthHeader(
        const std::string& signedHeadersString,
        const std::string& signature) const
{
    return
        std::string("AWS4-HMAC-SHA256 ") +
        "Credential=" + m_auth.access() + '/' +
            m_formattedTime.date() + "/" + m_region + "/s3/aws4_request, " +
        "SignedHeaders=" + signedHeadersString + ", " +
        "Signature=" + signature;
}




// These functions allow a caller to directly pass additional headers into
// their GET request.  This is only applicable when using the S3 driver
// directly, as these are not available through the Arbiter.

std::vector<char> S3::getBinary(std::string rawPath, Headers headers) const
{
    std::vector<char> data;
    if (!get(Arbiter::stripType(rawPath), Query(), headers, data))
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

S3::Resource::Resource(std::string baseUrl, std::string fullPath)
    : baseUrl(baseUrl)
    , bucket()
    , object()
{
    fullPath = Http::sanitize(fullPath);
    const std::size_t split(fullPath.find("/"));

    bucket = fullPath.substr(0, split);

    if (split != std::string::npos)
    {
        object = fullPath.substr(split + 1);
    }
}

std::string S3::Resource::buildPath(Query query) const
{
    const std::string queryString(getQueryString(query));
    return "https://" + bucket + baseUrl + object + queryString;
}

std::string S3::Resource::host() const
{
    return bucket + baseUrl.substr(0, baseUrl.size() - 1); // Pop slash.
}

S3::FormattedTime::FormattedTime()
    : m_date(formatTime(dateFormat))
    , m_time(formatTime(timeFormat))
{ }

std::string S3::FormattedTime::formatTime(const std::string& format) const
{
    std::time_t time(std::time(nullptr));
    std::vector<char> buf(80, 0);

    if (std::strftime(
                buf.data(),
                buf.size(),
                format.data(),
                std::gmtime(&time)))
    {
        return std::string(buf.data());
    }
    else
    {
        throw ArbiterError("Could not format time");
    }
}
} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


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

#ifndef ARBITER_EXTERNAL_JSON
#include <arbiter/third/json/json.hpp>
#endif

#endif



#ifdef ARBITER_EXTERNAL_JSON
#include <json/json.h>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
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
            throw ArbiterError(
                    "Data size check failed - got " + std::to_string(size) +
                    " of " + std::to_string(res.data().size()) + " bytes.");
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

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/dropbox.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/md5.cpp
// //////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <cstdlib>
#include <memory>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/md5.hpp>
#include <arbiter/util/macros.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{
namespace
{

const std::size_t blockSize(16);

struct Md5Context
{
    Md5Context() : data(), datalen(0), bitlen(0), state()
    {
        state[0] = 0x67452301;
        state[1] = 0xEFCDAB89;
        state[2] = 0x98BADCFE;
        state[3] = 0x10325476;
    }

    uint8_t data[64];
    uint32_t datalen;
    unsigned long long bitlen;
    uint32_t state[4];
};

void md5_transform(Md5Context *ctx, const uint8_t data[])
{
    uint32_t a, b, c, d, m[16], i, j;

    // MD5 specifies big endian byte order, but this implementation assumes a
    // little endian byte order CPU. Reverse all the bytes upon input, and
    // re-reverse them on output (in md5_final()).
    for (i = 0, j = 0; i < 16; ++i, j += 4)
    {
        m[i] =
            (data[j]) + (data[j + 1] << 8) +
            (data[j + 2] << 16) + (data[j + 3] << 24);
    }

    a = ctx->state[0];
    b = ctx->state[1];
    c = ctx->state[2];
    d = ctx->state[3];

    FF(a,b,c,d,m[0],  7,0xd76aa478);
    FF(d,a,b,c,m[1], 12,0xe8c7b756);
    FF(c,d,a,b,m[2], 17,0x242070db);
    FF(b,c,d,a,m[3], 22,0xc1bdceee);
    FF(a,b,c,d,m[4],  7,0xf57c0faf);
    FF(d,a,b,c,m[5], 12,0x4787c62a);
    FF(c,d,a,b,m[6], 17,0xa8304613);
    FF(b,c,d,a,m[7], 22,0xfd469501);
    FF(a,b,c,d,m[8],  7,0x698098d8);
    FF(d,a,b,c,m[9], 12,0x8b44f7af);
    FF(c,d,a,b,m[10],17,0xffff5bb1);
    FF(b,c,d,a,m[11],22,0x895cd7be);
    FF(a,b,c,d,m[12], 7,0x6b901122);
    FF(d,a,b,c,m[13],12,0xfd987193);
    FF(c,d,a,b,m[14],17,0xa679438e);
    FF(b,c,d,a,m[15],22,0x49b40821);

    GG(a,b,c,d,m[1],  5,0xf61e2562);
    GG(d,a,b,c,m[6],  9,0xc040b340);
    GG(c,d,a,b,m[11],14,0x265e5a51);
    GG(b,c,d,a,m[0], 20,0xe9b6c7aa);
    GG(a,b,c,d,m[5],  5,0xd62f105d);
    GG(d,a,b,c,m[10], 9,0x02441453);
    GG(c,d,a,b,m[15],14,0xd8a1e681);
    GG(b,c,d,a,m[4], 20,0xe7d3fbc8);
    GG(a,b,c,d,m[9],  5,0x21e1cde6);
    GG(d,a,b,c,m[14], 9,0xc33707d6);
    GG(c,d,a,b,m[3], 14,0xf4d50d87);
    GG(b,c,d,a,m[8], 20,0x455a14ed);
    GG(a,b,c,d,m[13], 5,0xa9e3e905);
    GG(d,a,b,c,m[2],  9,0xfcefa3f8);
    GG(c,d,a,b,m[7], 14,0x676f02d9);
    GG(b,c,d,a,m[12],20,0x8d2a4c8a);

    HH(a,b,c,d,m[5],  4,0xfffa3942);
    HH(d,a,b,c,m[8], 11,0x8771f681);
    HH(c,d,a,b,m[11],16,0x6d9d6122);
    HH(b,c,d,a,m[14],23,0xfde5380c);
    HH(a,b,c,d,m[1],  4,0xa4beea44);
    HH(d,a,b,c,m[4], 11,0x4bdecfa9);
    HH(c,d,a,b,m[7], 16,0xf6bb4b60);
    HH(b,c,d,a,m[10],23,0xbebfbc70);
    HH(a,b,c,d,m[13], 4,0x289b7ec6);
    HH(d,a,b,c,m[0], 11,0xeaa127fa);
    HH(c,d,a,b,m[3], 16,0xd4ef3085);
    HH(b,c,d,a,m[6], 23,0x04881d05);
    HH(a,b,c,d,m[9],  4,0xd9d4d039);
    HH(d,a,b,c,m[12],11,0xe6db99e5);
    HH(c,d,a,b,m[15],16,0x1fa27cf8);
    HH(b,c,d,a,m[2], 23,0xc4ac5665);

    II(a,b,c,d,m[0],  6,0xf4292244);
    II(d,a,b,c,m[7], 10,0x432aff97);
    II(c,d,a,b,m[14],15,0xab9423a7);
    II(b,c,d,a,m[5], 21,0xfc93a039);
    II(a,b,c,d,m[12], 6,0x655b59c3);
    II(d,a,b,c,m[3], 10,0x8f0ccc92);
    II(c,d,a,b,m[10],15,0xffeff47d);
    II(b,c,d,a,m[1], 21,0x85845dd1);
    II(a,b,c,d,m[8],  6,0x6fa87e4f);
    II(d,a,b,c,m[15],10,0xfe2ce6e0);
    II(c,d,a,b,m[6], 15,0xa3014314);
    II(b,c,d,a,m[13],21,0x4e0811a1);
    II(a,b,c,d,m[4],  6,0xf7537e82);
    II(d,a,b,c,m[11],10,0xbd3af235);
    II(c,d,a,b,m[2], 15,0x2ad7d2bb);
    II(b,c,d,a,m[9], 21,0xeb86d391);

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
}

void md5_update(Md5Context *ctx, const uint8_t data[], std::size_t len)
{
    for (std::size_t i(0); i < len; ++i) {
        ctx->data[ctx->datalen] = data[i];
        ctx->datalen++;
        if (ctx->datalen == 64) {
            md5_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

void md5_final(Md5Context *ctx, uint8_t hash[])
{
    std::size_t i(ctx->datalen);

    // Pad whatever data is left in the buffer.
    if (ctx->datalen < 56) {
        ctx->data[i++] = 0x80;
        while (i < 56)
            ctx->data[i++] = 0x00;
    }
    else if (ctx->datalen >= 56) {
        ctx->data[i++] = 0x80;
        while (i < 64)
            ctx->data[i++] = 0x00;
        md5_transform(ctx, ctx->data);
        memset(ctx->data, 0, 56);
    }

    // Append to the padding the total message's length in bits and transform.
    ctx->bitlen += ctx->datalen * 8;
    ctx->data[56] = static_cast<uint8_t>(ctx->bitlen);
    ctx->data[57] = static_cast<uint8_t>(ctx->bitlen >> 8);
    ctx->data[58] = static_cast<uint8_t>(ctx->bitlen >> 16);
    ctx->data[59] = static_cast<uint8_t>(ctx->bitlen >> 24);
    ctx->data[60] = static_cast<uint8_t>(ctx->bitlen >> 32);
    ctx->data[61] = static_cast<uint8_t>(ctx->bitlen >> 40);
    ctx->data[62] = static_cast<uint8_t>(ctx->bitlen >> 48);
    ctx->data[63] = static_cast<uint8_t>(ctx->bitlen >> 56);
    md5_transform(ctx, ctx->data);

    // Since this implementation uses little endian byte ordering and MD uses
    // big endian, reverse all the bytes when copying the final state to the
    // output hash.
    for (i = 0; i < 4; ++i) {
        hash[i]      = (ctx->state[0] >> (i * 8)) & 0x000000ff;
        hash[i + 4]  = (ctx->state[1] >> (i * 8)) & 0x000000ff;
        hash[i + 8]  = (ctx->state[2] >> (i * 8)) & 0x000000ff;
        hash[i + 12] = (ctx->state[3] >> (i * 8)) & 0x000000ff;
    }
}

} // unnamed namespace

std::string md5(const std::string& data)
{
    std::vector<char> out(blockSize, 0);

    Md5Context ctx;
    md5_update(
            &ctx,
            reinterpret_cast<const uint8_t*>(data.data()),
            data.size());
    md5_final(&ctx, reinterpret_cast<uint8_t*>(out.data()));

    return std::string(out.data(), out.size());
}

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/md5.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/sha256.cpp
// //////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <memory>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/sha256.hpp>
#include <arbiter/util/macros.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{
namespace
{

const std::size_t block(64);

const std::vector<uint32_t> k {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

struct Sha256Context
{
    Sha256Context() : data(), datalen(0), bitlen(0), state()
    {
        state[0] = 0x6a09e667;
        state[1] = 0xbb67ae85;
        state[2] = 0x3c6ef372;
        state[3] = 0xa54ff53a;
        state[4] = 0x510e527f;
        state[5] = 0x9b05688c;
        state[6] = 0x1f83d9ab;
        state[7] = 0x5be0cd19;
    }

    uint8_t data[64];
    uint32_t datalen;
    std::size_t bitlen;
    uint32_t state[8];
};

void sha256_transform(Sha256Context *ctx, const uint8_t data[])
{
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];

    for (i = 0, j = 0; i < 16; ++i, j += 4)
    {
        m[i] =
            (data[j    ] << 24) |
            (data[j + 1] << 16) |
            (data[j + 2] << 8 ) |
            (data[j + 3]);
    }

    for ( ; i < 64; ++i)
    {
        m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];
    }

    a = ctx->state[0];
    b = ctx->state[1];
    c = ctx->state[2];
    d = ctx->state[3];
    e = ctx->state[4];
    f = ctx->state[5];
    g = ctx->state[6];
    h = ctx->state[7];

    for (i = 0; i < 64; ++i)
    {
        t1 = h + EP1(e) + CH(e,f,g) + k[i] + m[i];
        t2 = EP0(a) + MAJ(a,b,c);
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
    ctx->state[5] += f;
    ctx->state[6] += g;
    ctx->state[7] += h;
}

void sha256_update(Sha256Context *ctx, const uint8_t data[], std::size_t len)
{
    uint32_t i;

    for (i = 0; i < len; ++i)
    {
        ctx->data[ctx->datalen] = data[i];

        if (++ctx->datalen == 64)
        {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

void sha256_final(Sha256Context *ctx, uint8_t hash[])
{
    uint32_t i(ctx->datalen);

    // Pad whatever data is left in the buffer.
    if (ctx->datalen < 56)
    {
        ctx->data[i++] = 0x80;

        while (i < 56)
        {
            ctx->data[i++] = 0x00;
        }
    }
    else
    {
        ctx->data[i++] = 0x80;

        while (i < 64)
        {
            ctx->data[i++] = 0x00;
        }

        sha256_transform(ctx, ctx->data);
        memset(ctx->data, 0, 56);
    }

    // Append to the padding the total message's length in bits and transform.
    ctx->bitlen += ctx->datalen * 8;
    ctx->data[63] = ctx->bitlen;
    ctx->data[62] = ctx->bitlen >> 8;
    ctx->data[61] = ctx->bitlen >> 16;
    ctx->data[60] = ctx->bitlen >> 24;
    ctx->data[59] = ctx->bitlen >> 32;
    ctx->data[58] = ctx->bitlen >> 40;
    ctx->data[57] = ctx->bitlen >> 48;
    ctx->data[56] = ctx->bitlen >> 56;
    sha256_transform(ctx, ctx->data);

    // Since this implementation uses little endian byte ordering and SHA uses
    // big endian, reverse all the bytes when copying the final state to the
    // output hash.
    for (i = 0; i < 4; ++i)
    {
        hash[i]      = (ctx->state[0] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 4]  = (ctx->state[1] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 8]  = (ctx->state[2] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0x000000ff;
    }
}

} // unnamed namespace

std::vector<char> sha256(const std::vector<char>& data)
{
    std::vector<char> out(32, 0);

    Sha256Context ctx;
    sha256_update(
            &ctx,
            reinterpret_cast<const uint8_t*>(data.data()),
            data.size());
    sha256_final(&ctx, reinterpret_cast<uint8_t*>(out.data()));

    return out;
}

std::string sha256(const std::string& data)
{
    const std::vector<char> v(data.begin(), data.end());
    const std::vector<char> result(sha256(v));
    return std::string(result.data(), result.size());
}

std::string hmacSha256(const std::string& rawKey, const std::string& data)
{
    std::string key(rawKey);

    if (key.size() > block) key = sha256(key);
    if (key.size() < block) key.insert(key.end(), block - key.size(), 0);

    std::string okeypad(block, 0x5c);
    std::string ikeypad(block, 0x36);

    for (std::size_t i(0); i < block; ++i)
    {
        okeypad[i] ^= key[i];
        ikeypad[i] ^= key[i];
    }

    return sha256(okeypad + sha256(ikeypad + data));
}

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/sha256.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/transforms.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/transforms.hpp>
#endif

#include <cstdint>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{
namespace
{
    const std::string base64Vals(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/");

    const std::string hexVals("0123456789abcdef");
} // unnamed namespace

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

std::string encodeBase64(const std::string& input)
{
    return encodeBase64(std::vector<char>(input.begin(), input.end()));
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

std::string encodeAsHex(const std::string& input)
{
    return encodeAsHex(std::vector<char>(input.begin(), input.end()));
}

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/transforms.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/util.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/util.hpp>

#include <arbiter/arbiter.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace util
{

std::string getBasename(const std::string fullPath)
{
    std::string result(fullPath);

    std::string stripped(Arbiter::stripType(fullPath));

    for (std::size_t i(0); i < 2; ++i)
    {
        // Pop trailing asterisk, or double-trailing-asterisks for both non- and
        // recursive globs.
        if (!stripped.empty() && stripped.back() == '*') stripped.pop_back();
    }

    // Pop trailing slash, in which case the result is the innermost directory.
    while (!stripped.empty() && isSlash(stripped.back())) stripped.pop_back();

    // Now do the real slash searching.
    const std::size_t pos(stripped.rfind('/'));

    if (pos != std::string::npos)
    {
        const std::string sub(stripped.substr(pos + 1));
        if (!sub.empty()) result = sub;
    }

    return result;
}

} // namespace util
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/util.cpp
// //////////////////////////////////////////////////////////////////////





