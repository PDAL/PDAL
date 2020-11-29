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
#include <arbiter/util/sha256.hpp>
#include <arbiter/util/json.hpp>
#include <arbiter/util/transforms.hpp>
#include <arbiter/util/util.hpp>
#endif

#include <algorithm>
#include <cstdlib>
#include <sstream>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace
{
    const std::string delimiter("://");

#ifdef ARBITER_CURL
    const std::size_t concurrentHttpReqs(32);
    const std::size_t httpRetryCount(8);
#endif

    json getConfig(const std::string& s)
    {
        json in(s.size() ? json::parse(s) : json::object());

        json config;
        std::string path("~/.arbiter/config.json");

        if      (auto p = env("ARBITER_CONFIG_FILE")) path = *p;
        else if (auto p = env("ARBITER_CONFIG_PATH")) path = *p;

        if (auto data = drivers::Fs().tryGet(path)) config = json::parse(*data);

        if (in.is_null()) in = json::object();
        if (config.is_null()) config = json::object();

        return merge(in, config);
    }
}

Arbiter::Arbiter() : Arbiter(json().dump()) { }

Arbiter::Arbiter(const std::string s)
    : m_drivers()
#ifdef ARBITER_CURL
    , m_pool(
            new http::Pool(
                concurrentHttpReqs,
                httpRetryCount,
                getConfig(s).dump()))
#endif
{
    using namespace drivers;

    const json c(getConfig(s));

    if (auto d = Fs::create())
    {
        m_drivers[d->type()] = std::move(d);
    }

    if (auto d = Test::create())
    {
        m_drivers[d->type()] = std::move(d);
    }

#ifdef ARBITER_CURL
    if (auto d = Http::create(*m_pool))
    {
        m_drivers[d->type()] = std::move(d);
    }

    if (auto d = Https::create(*m_pool))
    {
        m_drivers[d->type()] = std::move(d);
    }

    {
        auto dlist(S3::create(*m_pool, c.value("s3", json()).dump()));
        for (auto& d : dlist) m_drivers[d->type()] = std::move(d);
    }

    // Credential-based drivers should probably all do something similar to the
    // S3 driver to support multiple profiles.
    if (auto d = Dropbox::create(*m_pool, c.value("dropbox", json()).dump()))
    {
        m_drivers[d->type()] = std::move(d);
    }

#ifdef ARBITER_OPENSSL
    if (auto d = Google::create(*m_pool, c.value("gs", json()).dump()))
    {
        m_drivers[d->type()] = std::move(d);
    }
#endif

#endif
}

bool Arbiter::hasDriver(const std::string path) const
{
    return m_drivers.count(getProtocol(path));
}

void Arbiter::addDriver(const std::string type, std::unique_ptr<Driver> driver)
{
    if (!driver) throw ArbiterError("Cannot add empty driver for " + type);
    m_drivers[type] = std::move(driver);
}

std::string Arbiter::get(const std::string path) const
{
    return getDriver(path).get(stripProtocol(path));
}

std::vector<char> Arbiter::getBinary(const std::string path) const
{
    return getDriver(path).getBinary(stripProtocol(path));
}

std::unique_ptr<std::string> Arbiter::tryGet(std::string path) const
{
    return getDriver(path).tryGet(stripProtocol(path));
}

std::unique_ptr<std::vector<char>> Arbiter::tryGetBinary(std::string path) const
{
    return getDriver(path).tryGetBinary(stripProtocol(path));
}

std::size_t Arbiter::getSize(const std::string path) const
{
    return getDriver(path).getSize(stripProtocol(path));
}

std::unique_ptr<std::size_t> Arbiter::tryGetSize(const std::string path) const
{
    return getDriver(path).tryGetSize(stripProtocol(path));
}

void Arbiter::put(const std::string path, const std::string& data) const
{
    return getDriver(path).put(stripProtocol(path), data);
}

void Arbiter::put(const std::string path, const std::vector<char>& data) const
{
    return getDriver(path).put(stripProtocol(path), data);
}

std::string Arbiter::get(
        const std::string path,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).get(stripProtocol(path), headers, query);
}

std::unique_ptr<std::string> Arbiter::tryGet(
        const std::string path,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).tryGet(stripProtocol(path), headers, query);
}

std::vector<char> Arbiter::getBinary(
        const std::string path,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).getBinary(stripProtocol(path), headers, query);
}

std::unique_ptr<std::vector<char>> Arbiter::tryGetBinary(
        const std::string path,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).tryGetBinary(stripProtocol(path), headers, query);
}

void Arbiter::put(
        const std::string path,
        const std::string& data,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).put(stripProtocol(path), data, headers, query);
}

void Arbiter::put(
        const std::string path,
        const std::vector<char>& data,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver(path).put(stripProtocol(path), data, headers, query);
}

void Arbiter::copy(
        const std::string src,
        const std::string dst,
        const bool verbose) const
{
    if (src.empty()) throw ArbiterError("Cannot copy from empty source");
    if (dst.empty()) throw ArbiterError("Cannot copy to empty destination");

    // Globify the source path if it's a directory.  In this case, the source
    // already ends with a slash.
    const std::string srcToResolve(src + (isDirectory(src) ? "**" : ""));

    if (srcToResolve.back() != '*')
    {
        // The source is a single file.
        copyFile(src, dst, verbose);
    }
    else
    {
        // We'll need this to mirror the directory structure in the output.
        // All resolved paths will contain this common prefix, so we can
        // determine any nested paths from recursive resolutions by stripping
        // that common portion.
        const Endpoint& srcEndpoint(getEndpoint(stripPostfixing(src)));
        const std::string commonPrefix(srcEndpoint.prefixedRoot());

        const Endpoint dstEndpoint(getEndpoint(dst));

        if (srcEndpoint.prefixedRoot() == dstEndpoint.prefixedRoot())
        {
            throw ArbiterError("Cannot copy directory to itself");
        }

        int i(0);
        const auto paths(resolve(srcToResolve, verbose));

        for (const auto& path : paths)
        {
            const std::string subpath(path.substr(commonPrefix.size()));

            if (verbose)
            {
                std::cout <<
                    ++i << " / " << paths.size() << ": " <<
                    path << " -> " << dstEndpoint.prefixedFullPath(subpath) <<
                    std::endl;
            }

            if (dstEndpoint.isLocal())
            {
                mkdirp(getDirname(dstEndpoint.fullPath(subpath)));
            }

            dstEndpoint.put(subpath, getBinary(path));
        }
    }
}

void Arbiter::copyFile(
        const std::string file,
        std::string dst,
        const bool verbose) const
{
    if (dst.empty()) throw ArbiterError("Cannot copy to empty destination");

    const Endpoint dstEndpoint(getEndpoint(dst));

    if (isDirectory(dst))
    {
        // If the destination is a directory, maintain the basename of the
        // source file.
        dst += getBasename(file);
    }

    if (verbose) std::cout << file << " -> " << dst << std::endl;

    if (dstEndpoint.isLocal()) mkdirp(getDirname(dst));

    if (getEndpoint(file).type() == dstEndpoint.type())
    {
        // If this copy is within the same driver domain, defer to the
        // hopefully specialized copy method.
        getDriver(file).copy(stripProtocol(file), stripProtocol(dst));
    }
    else
    {
        // Otherwise do a GET/PUT for the copy.
        put(dst, getBinary(file));
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

bool Arbiter::exists(const std::string path) const
{
    return tryGetSize(path).get() != nullptr;
}

bool Arbiter::isHttpDerived(const std::string path) const
{
    return tryGetHttpDriver(path) != nullptr;
}

std::vector<std::string> Arbiter::resolve(
        const std::string path,
        const bool verbose) const
{
    return getDriver(path).resolve(stripProtocol(path), verbose);
}

Endpoint Arbiter::getEndpoint(const std::string root) const
{
    return Endpoint(getDriver(root), stripProtocol(root));
}

const Driver& Arbiter::getDriver(const std::string path) const
{
    const auto type(getProtocol(path));

    if (!m_drivers.count(type))
    {
        throw ArbiterError("No driver for " + path);
    }

    return *m_drivers.at(type);
}

const drivers::Http* Arbiter::tryGetHttpDriver(const std::string path) const
{
    return dynamic_cast<const drivers::Http*>(&getDriver(path));
}

const drivers::Http& Arbiter::getHttpDriver(const std::string path) const
{
    if (auto d = tryGetHttpDriver(path)) return *d;
    else throw ArbiterError("Cannot get driver for " + path + " as HTTP");
}

LocalHandle Arbiter::getLocalHandle(
        const std::string path,
        const Endpoint& tempEndpoint) const
{
    const Endpoint fromEndpoint(getEndpoint(getDirname(path)));
    return fromEndpoint.getLocalHandle(getBasename(path));
}

LocalHandle Arbiter::getLocalHandle(
        const std::string path,
        std::string tempPath) const
{
    if (tempPath.empty()) tempPath = getTempPath();
    return getLocalHandle(path, getEndpoint(tempPath));
}

LocalHandle Arbiter::getLocalHandle(
        const std::string path,
        http::Headers headers,
        http::Query query) const
{
    const Endpoint fromEndpoint(getEndpoint(getDirname(path)));
    return fromEndpoint.getLocalHandle(getBasename(path), headers, query);
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

std::string Driver::get(const std::string path) const
{
    const std::vector<char> data(getBinary(path));
    return std::string(data.begin(), data.end());
}

std::unique_ptr<std::string> Driver::tryGet(const std::string path) const
{
    std::unique_ptr<std::string> result;
    std::unique_ptr<std::vector<char>> data(tryGetBinary(path));
    if (data) result.reset(new std::string(data->begin(), data->end()));
    return result;
}

std::vector<char> Driver::getBinary(std::string path) const
{
    std::vector<char> data;
    if (!get(path, data)) throw ArbiterError("Could not read file " + path);
    return data;
}

std::unique_ptr<std::vector<char>> Driver::tryGetBinary(std::string path) const
{
    std::unique_ptr<std::vector<char>> data(new std::vector<char>());
    if (!get(path, *data)) data.reset();
    return data;
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

void Driver::copy(std::string src, std::string dst) const
{
    put(dst, getBinary(src));
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
        if (isRemote()) path = type() + "://" + path;
        else path = expandTilde(path);

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

#include <algorithm>
#include <fstream>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/endpoint.hpp>

#include <arbiter/arbiter.hpp>
#include <arbiter/driver.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/util/sha256.hpp>
#include <arbiter/util/transforms.hpp>
#include <arbiter/util/util.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace
{
    constexpr std::size_t mb = 1024 * 1024;
    const std::size_t chunkSize = 10 * mb;
    const auto streamFlags(
        std::ofstream::binary |
        std::ofstream::out |
        std::ofstream::app);
    std::string postfixSlash(std::string path)
    {
        if (!path.empty() && path.back() != '/') path.push_back('/');
        return path;
    }
}

Endpoint::Endpoint(const Driver& driver, const std::string root)
    : m_driver(&driver)
    , m_root(expandTilde(postfixSlash(root)))
{ }

std::string Endpoint::root() const
{
    return m_root;
}

std::string Endpoint::prefixedRoot() const
{
    return softPrefix() + root();
}

std::string Endpoint::type() const
{
    return m_driver->type();
}

bool Endpoint::isRemote() const
{
    return m_driver->isRemote();
}

bool Endpoint::isLocal() const
{
    return !isRemote();
}

bool Endpoint::isHttpDerived() const
{
    return tryGetHttpDriver() != nullptr;
}

LocalHandle Endpoint::getLocalHandle(
        const std::string subpath,
        http::Headers headers,
        http::Query query) const
{
    if (isRemote())
    {
        const std::string tmp(getTempPath());
        const auto ext(getExtension(subpath));
        const std::string basename(
                std::to_string(randomNumber()) +
                (ext.size() ? "." + ext : ""));

        const std::string local(tmp + basename);
        if (isHttpDerived())
        {
            if (auto fileSize = tryGetSize(subpath))
            {
                std::ofstream stream(local, streamFlags);
                if (!stream.good())
                {
                    throw ArbiterError("Unable to create local handle");
                }

                for (std::size_t pos(0); pos < *fileSize; pos += chunkSize)
                {
                    const std::size_t end =
                        (std::min)(pos + chunkSize, *fileSize);
                    const std::string range("bytes=" +
                        std::to_string(pos) + "-" +
                        std::to_string(end - 1));
                    headers["Range"] = range;
                    const auto data(getBinary(subpath, headers, query));
                    stream.write(data.data(), data.size());

                    if (!stream.good())
                    {
                        throw ArbiterError("Unable to write local handle");
                    }
                }
            }
            else
            {
                drivers::Fs fs;
                fs.put(local, getBinary(subpath, headers, query));
            }
        }
        else
        {
            drivers::Fs fs;
            fs.put(local, getBinary(subpath));
        }

        return LocalHandle(local, true);
    }
    else
    {
        return LocalHandle(expandTilde(fullPath(subpath)), false);
    }
}

std::string Endpoint::get(const std::string subpath) const
{
    return m_driver->get(fullPath(subpath));
}

std::unique_ptr<std::string> Endpoint::tryGet(const std::string subpath)
    const
{
    return m_driver->tryGet(fullPath(subpath));
}

std::vector<char> Endpoint::getBinary(const std::string subpath) const
{
    return m_driver->getBinary(fullPath(subpath));
}

std::unique_ptr<std::vector<char>> Endpoint::tryGetBinary(
        const std::string subpath) const
{
    return m_driver->tryGetBinary(fullPath(subpath));
}

std::size_t Endpoint::getSize(const std::string subpath) const
{
    return m_driver->getSize(fullPath(subpath));
}

std::unique_ptr<std::size_t> Endpoint::tryGetSize(
        const std::string subpath) const
{
    return m_driver->tryGetSize(fullPath(subpath));
}

void Endpoint::put(const std::string subpath, const std::string& data) const
{
    m_driver->put(fullPath(subpath), data);
}

void Endpoint::put(
        const std::string subpath,
        const std::vector<char>& data) const
{
    m_driver->put(fullPath(subpath), data);
}

std::string Endpoint::get(
        const std::string subpath,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver().get(fullPath(subpath), headers, query);
}

std::unique_ptr<std::string> Endpoint::tryGet(
        const std::string subpath,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver().tryGet(fullPath(subpath), headers, query);
}

std::vector<char> Endpoint::getBinary(
        const std::string subpath,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver().getBinary(fullPath(subpath), headers, query);
}

std::unique_ptr<std::vector<char>> Endpoint::tryGetBinary(
        const std::string subpath,
        const http::Headers headers,
        const http::Query query) const
{
    return getHttpDriver().tryGetBinary(fullPath(subpath), headers, query);
}

void Endpoint::put(
        const std::string path,
        const std::string& data,
        const http::Headers headers,
        const http::Query query) const
{
    getHttpDriver().put(path, data, headers, query);
}

void Endpoint::put(
        const std::string path,
        const std::vector<char>& data,
        const http::Headers headers,
        const http::Query query) const
{
    getHttpDriver().put(path, data, headers, query);
}

http::Response Endpoint::httpGet(
        std::string path,
        http::Headers headers,
        http::Query query,
        const std::size_t reserve) const
{
    return getHttpDriver().internalGet(fullPath(path), headers, query, reserve);
}

http::Response Endpoint::httpPut(
        std::string path,
        const std::vector<char>& data,
        http::Headers headers,
        http::Query query) const
{
    return getHttpDriver().internalPut(fullPath(path), data, headers, query);
}

http::Response Endpoint::httpHead(
        std::string path,
        http::Headers headers,
        http::Query query) const
{
    return getHttpDriver().internalHead(fullPath(path), headers, query);
}

http::Response Endpoint::httpPost(
        std::string path,
        const std::vector<char>& data,
        http::Headers headers,
        http::Query query) const
{
    return getHttpDriver().internalPost(fullPath(path), data, headers, query);
}

std::string Endpoint::fullPath(const std::string& subpath) const
{
    return m_root + subpath;
}

std::string Endpoint::prefixedFullPath(const std::string& subpath) const
{
     return softPrefix() + fullPath(subpath);
}

Endpoint Endpoint::getSubEndpoint(std::string subpath) const
{
    return Endpoint(*m_driver, m_root + subpath);
}

std::string Endpoint::softPrefix() const
{
    return isRemote() ? type() + "://" : "";
}

const drivers::Http* Endpoint::tryGetHttpDriver() const
{
    return dynamic_cast<const drivers::Http*>(m_driver);
}

const drivers::Http& Endpoint::getHttpDriver() const
{
    if (auto d = tryGetHttpDriver()) return *d;
    else throw ArbiterError("Cannot get driver of type " + type() + " as HTTP");
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
#include <arbiter/util/util.hpp>
#endif

#ifndef ARBITER_WINDOWS
#include <glob.h>
#include <sys/stat.h>
#else
#define UNICODE
#include <Shlwapi.h>
#include <iterator>
#include <locale>
#include <codecvt>
#include <windows.h>
#include <direct.h>
#endif

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <ios>
#include <istream>

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

    std::string getHome()
    {
        std::string s;

#ifndef ARBITER_WINDOWS
        if (auto home = env("HOME")) s = *home;
#else
        if (auto userProfile = env("USERPROFILE"))
        {
            s = *userProfile;
        }
        else
        {
            auto homeDrive(env("HOMEDRIVE"));
            auto homePath(env("HOMEPATH"));

            if (homeDrive && homePath) s = *homeDrive + *homePath;
        }
#endif

        return s;
    }
}

namespace drivers
{

std::unique_ptr<Fs> Fs::create()
{
    return std::unique_ptr<Fs>(new Fs());
}

std::unique_ptr<std::size_t> Fs::tryGetSize(std::string path) const
{
    std::unique_ptr<std::size_t> size;

    path = expandTilde(path);

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

    path = expandTilde(path);
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
    path = expandTilde(path);
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

void Fs::copy(std::string src, std::string dst) const
{
    src = expandTilde(src);
    dst = expandTilde(dst);

    std::ifstream instream(src, std::ifstream::in | std::ifstream::binary);
    if (!instream.good())
    {
        throw ArbiterError("Could not open " + src + " for reading");
    }
    instream >> std::noskipws;

    std::ofstream outstream(dst, binaryTruncMode);
    if (!outstream.good())
    {
        throw ArbiterError("Could not open " + dst + " for writing");
    }

    outstream << instream.rdbuf();
}

std::vector<std::string> Fs::glob(std::string path, bool verbose) const
{
    return arbiter::glob(path);
}

} // namespace drivers


bool mkdirp(std::string raw)
{
    const std::string dir(([&raw]()
    {
        std::string s(expandTilde(raw));

        // Remove consecutive slashes.  For Windows, we'll need to be careful
        // not to remove drive letters like C:\\.
        const auto end = std::unique(s.begin(), s.end(), [](char l, char r)
        {
            return isSlash(l) && isSlash(r);
        });

        s = std::string(s.begin(), end);
        if (s.size() && isSlash(s.back())) s.pop_back();
        return s;
    })());

    auto it(dir.begin());
    const auto end(dir.cend());

    do
    {
        it = std::find_if(++it, end, isSlash);

        const std::string cur(dir.begin(), it);
#ifndef ARBITER_WINDOWS
        const bool err(::mkdir(cur.c_str(), 0777));
        if (err && errno != EEXIST) return false;
#else
        // Use CreateDirectory instead of _mkdir; it is more reliable when creating directories on a drive other than the working path.

        std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
		const std::wstring wide(converter.from_bytes(cur));
		const bool err(::CreateDirectoryW(wide.c_str(), NULL));
        if (err && ::GetLastError() != ERROR_ALREADY_EXISTS) return false;
#endif
    }
    while (it != end);

    return true;

}

bool remove(std::string filename)
{
    filename = expandTilde(filename);

    return ::remove(filename.c_str()) == 0;
}

namespace
{
    struct Globs
    {
        std::vector<std::string> files;
        std::vector<std::string> dirs;
    };

template<typename C>
	std::basic_string<C> remove_dups(std::basic_string<C> s, C c)
	{
		C cc[3] = { c, c };
		auto pos = s.find(cc);
		while (pos != s.npos) {
			s.erase(pos, 1);
			pos = s.find(cc, pos + 1);
		}
		return s;
	}

#ifdef ARBITER_WINDOWS
	bool icase_wchar_cmp(wchar_t a, wchar_t b)
	{
		return std::toupper(a, std::locale()) == std::toupper(b, std::locale());
	}


	bool icase_cmp(std::wstring const& s1, std::wstring const& s2)
	{
		return (s1.size() == s2.size()) &&
			std::equal(s1.begin(), s1.end(), s2.begin(),
				icase_wchar_cmp);
	}
#endif

    Globs globOne(std::string path)
    {
        Globs results;

#ifndef ARBITER_WINDOWS
        glob_t buffer;
        struct stat info;

        ::glob(path.c_str(), GLOB_NOSORT | GLOB_MARK, 0, &buffer);

        for (std::size_t i(0); i < buffer.gl_pathc; ++i)
        {
            const std::string val(buffer.gl_pathv[i]);

            if (stat(val.c_str(), &info) == 0)
            {
                if (S_ISREG(info.st_mode)) results.files.push_back(val);
                else if (S_ISDIR(info.st_mode)) results.dirs.push_back(val);
            }
            else
            {
                throw ArbiterError("Error globbing - POSIX stat failed");
            }
        }

        globfree(&buffer);
#else
		std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
		std::wstring wide(converter.from_bytes(path));

		WIN32_FIND_DATAW data{};
		LPCWSTR fname = wide.c_str();
        HANDLE hFind(INVALID_HANDLE_VALUE);
		hFind = FindFirstFileW(fname, &data);

		if (hFind == (HANDLE)-1) return results; // bad filename

        if (hFind != INVALID_HANDLE_VALUE )
        {
            do
            {
				if (icase_cmp(std::wstring(data.cFileName), L".") ||
					icase_cmp(std::wstring(data.cFileName), L".."))
					continue;

				std::vector<wchar_t> buf(MAX_PATH);
				wide.erase(std::remove(wide.begin(), wide.end(), '*'), wide.end());

				std::replace(wide.begin(), wide.end(), '\\', '/');

				std::copy(wide.begin(), wide.end(), buf.begin()	);
                BOOL appended = PathAppendW(buf.data(), data.cFileName);

				std::wstring output(buf.data(), wcslen( buf.data()));

                // Erase any \'s
                output.erase(std::remove(output.begin(), output.end(), '\\'), output.end());

                if (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
                {
                    results.dirs.push_back(converter.to_bytes(output));

                    output.append(L"/*");
                    Globs more = globOne(converter.to_bytes(output));
                    std::copy(more.dirs.begin(), more.dirs.end(), std::back_inserter(results.dirs));
                    std::copy(more.files.begin(), more.files.end(), std::back_inserter(results.files));

                }

				results.files.push_back(
					converter.to_bytes(output));
            }
            while (FindNextFileW(hFind, &data));
        }
		FindClose(hFind);
#endif

        return results;
    }

    std::vector<std::string> walk(std::string dir)
    {
        std::vector<std::string> paths;
        paths.push_back(dir);

        for (const auto& d : globOne(dir + '*').dirs)
        {
            const auto next(walk(d));
            paths.insert(paths.end(), next.begin(), next.end());
        }

        return paths;
    }
}

std::vector<std::string> glob(std::string path)
{
    std::vector<std::string> results;

    path = expandTilde(path);

    if (path.find('*') == std::string::npos)
    {
        results.push_back(path);
        return results;
    }

    std::vector<std::string> dirs;

    const std::size_t recPos(path.find("**"));
    if (recPos != std::string::npos)
    {
        // Convert this recursive glob into multiple non-recursive ones.
        const auto pre(path.substr(0, recPos));     // Cut off before the '*'.
        const auto post(path.substr(recPos + 1));   // Includes the second '*'.

        for (const std::string& d : walk(pre))
            dirs.push_back(d + post);
    }
    else
    {
        dirs.push_back(path);
    }

    for (const auto& p : dirs)
    {
        Globs globs(globOne(p));
        results.insert(results.end(), globs.files.begin(), globs.files.end());
    }

    return results;
}

std::string expandTilde(std::string in)
{
    std::string out(in);
    if (!in.empty() && in.front() == '~')
    {
        const std::string home = getHome();
        if (home.empty()) throw ArbiterError("No home directory found");
        out = home + in.substr(1);
    }

    return out;
}

std::string getTempPath()
{
    std::string tmp;
#ifndef ARBITER_WINDOWS
    if (const auto t = env("TMPDIR"))         tmp = *t;
    else if (const auto t = env("TMP"))       tmp = *t;
    else if (const auto t = env("TEMP"))      tmp = *t;
    else if (const auto t = env("TEMPDIR"))   tmp = *t;
    else tmp = "/tmp";
#else
    std::vector<char> path(MAX_PATH, '\0');
    if (GetTempPath(MAX_PATH, path.data())) tmp.assign(path.data());
#endif

    if (tmp.empty()) throw ArbiterError("Could not find a temp path.");
    if (tmp.back() != '/') tmp += '/';
    return tmp;
}

LocalHandle::LocalHandle(const std::string localPath, const bool isRemote)
    : m_localPath(expandTilde(localPath))
    , m_erase(isRemote)
{ }

LocalHandle::~LocalHandle()
{
    if (m_erase) remove(expandTilde(m_localPath));
}

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
namespace drivers
{

using namespace http;

Http::Http(Pool& pool)
    : m_pool(pool)
{
#ifndef ARBITER_CURL
    throw ArbiterError("Cannot create HTTP driver - no curl support was built");
#endif
}

std::unique_ptr<Http> Http::create(Pool& pool)
{
    return std::unique_ptr<Http>(new Http(pool));
}

std::unique_ptr<std::size_t> Http::tryGetSize(std::string path) const
{
    std::unique_ptr<std::size_t> size;

    auto http(m_pool.acquire());
    Response res(http.head(typedPath(path)));

    if (res.ok() && res.headers().count("Content-Length"))
    {
        const std::string& str(res.headers().at("Content-Length"));
        size.reset(new std::size_t(std::stoul(str)));
    }

    return size;
}

std::string Http::get(
        std::string path,
        Headers headers,
        Query query) const
{
    const auto data(getBinary(path, headers, query));
    return std::string(data.begin(), data.end());
}

std::unique_ptr<std::string> Http::tryGet(
        std::string path,
        Headers headers,
        Query query) const
{
    std::unique_ptr<std::string> result;
    auto data(tryGetBinary(path, headers, query));
    if (data) result.reset(new std::string(data->begin(), data->end()));
    return result;
}

std::vector<char> Http::getBinary(
        std::string path,
        Headers headers,
        Query query) const
{
    std::vector<char> data;
    if (!get(path, data, headers, query))
    {
        throw ArbiterError("Could not read from " + path);
    }
    return data;
}

std::unique_ptr<std::vector<char>> Http::tryGetBinary(
        std::string path,
        Headers headers,
        Query query) const
{
    std::unique_ptr<std::vector<char>> data(new std::vector<char>());
    if (!get(path, *data, headers, query)) data.reset();
    return data;
}

void Http::put(
        std::string path,
        const std::string& data,
        const Headers headers,
        const Query query) const
{
    put(path, std::vector<char>(data.begin(), data.end()), headers, query);
}

bool Http::get(
        std::string path,
        std::vector<char>& data,
        const Headers headers,
        const Query query) const
{
    bool good(false);

    auto http(m_pool.acquire());
    Response res(http.get(typedPath(path), headers, query));

    if (res.ok())
    {
        data = res.data();
        good = true;
    }

    return good;
}

void Http::put(
        const std::string path,
        const std::vector<char>& data,
        const Headers headers,
        const Query query) const
{
    auto http(m_pool.acquire());

    if (!http.put(typedPath(path), data, headers, query).ok())
    {
        throw ArbiterError("Couldn't HTTP PUT to " + path);
    }
}

void Http::post(
        const std::string path,
        const std::string& data,
        const Headers h,
        const Query q) const
{
    return post(path, std::vector<char>(data.begin(), data.end()), h, q);
}

void Http::post(
        const std::string path,
        const std::vector<char>& data,
        const Headers headers,
        const Query query) const
{
    auto http(m_pool.acquire());
    auto res(http.post(typedPath(path), data, headers, query));

    if (!res.ok())
    {
        std::cout << res.str() << std::endl;
        throw ArbiterError("Couldn't HTTP POST to " + path);
    }
}

Response Http::internalGet(
        const std::string path,
        const Headers headers,
        const Query query,
        const std::size_t reserve) const
{
    return m_pool.acquire().get(typedPath(path), headers, query, reserve);
}

Response Http::internalPut(
        const std::string path,
        const std::vector<char>& data,
        const Headers headers,
        const Query query) const
{
    return m_pool.acquire().put(typedPath(path), data, headers, query);
}

Response Http::internalHead(
        const std::string path,
        const Headers headers,
        const Query query) const
{
    return m_pool.acquire().head(typedPath(path), headers, query);
}

Response Http::internalPost(
        const std::string path,
        const std::vector<char>& data,
        Headers headers,
        const Query query) const
{
    if (!headers.count("Content-Length"))
    {
        headers["Content-Length"] = std::to_string(data.size());
    }
    return m_pool.acquire().post(typedPath(path), data, headers, query);
}

std::string Http::typedPath(const std::string& p) const
{
    if (getProtocol(p) != "file") return p;
    else return type() + "://" + p;
}

} // namespace drivers

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
#include <sstream>
#include <thread>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/third/xml/xml.hpp>
#include <arbiter/util/ini.hpp>
#include <arbiter/util/json.hpp>
#include <arbiter/util/md5.hpp>
#include <arbiter/util/sha256.hpp>
#include <arbiter/util/transforms.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using namespace internal;

namespace
{
#ifdef ARBITER_CURL
    // Re-fetch credentials when there are less than 4 minutes remaining.  New
    // ones are guaranteed by AWS to be available within 5 minutes remaining.
    constexpr int64_t reauthSeconds(60 * 4);
#endif

    // See:
    // https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/iam-roles-for-amazon-ec2.html
    const std::string credIp("169.254.169.254/");
    const std::string credBase(
            credIp + "latest/meta-data/iam/security-credentials/");

    std::string line(const std::string& data) { return data + "\n"; }
    const std::vector<char> empty;

    typedef Xml::xml_node<> XmlNode;
    const std::string badResponse("Unexpected contents in AWS response");

    std::string toLower(const std::string& in)
    {
        return std::accumulate(
                in.begin(),
                in.end(),
                std::string(),
                [](const std::string& out, const char c) -> std::string
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
                [](const std::string& out, const char c) -> std::string
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
}

namespace drivers
{

using namespace http;

S3::S3(
        Pool& pool,
        std::string profile,
        std::unique_ptr<Auth> auth,
        std::unique_ptr<Config> config)
    : Http(pool)
    , m_profile(profile)
    , m_auth(std::move(auth))
    , m_config(std::move(config))
{ }

std::vector<std::unique_ptr<S3>> S3::create(Pool& pool, const std::string s)
{
    std::vector<std::unique_ptr<S3>> result;

    const json config(s.size() ? json::parse(s) : json());

    if (config.is_array())
    {
        for (const json& curr : config)
        {
            if (auto s = createOne(pool, curr.dump()))
            {
                result.push_back(std::move(s));
            }
        }
    }
    else if (auto s = createOne(pool, config.dump()))
    {
        result.push_back(std::move(s));
    }

    return result;
}

std::unique_ptr<S3> S3::createOne(Pool& pool, const std::string s)
{
    const json j(s.size() ? json::parse(s) : json());
    const std::string profile(extractProfile(j.dump()));

    auto auth(Auth::create(j.dump(), profile));
    if (!auth) return std::unique_ptr<S3>();

    std::unique_ptr<Config> config(new Config(j.dump(), profile));
    auto s3 = makeUnique<S3>(pool, profile, std::move(auth), std::move(config));
    return s3;
}

std::string S3::extractProfile(const std::string s)
{
    const json config(s.size() ? json::parse(s) : json());

    if (
            !config.is_null() &&
            config.count("profile") &&
            config["profile"].get<std::string>().size())
    {
        return config["profile"].get<std::string>();
    }

    if (auto p = env("AWS_PROFILE")) return *p;
    if (auto p = env("AWS_DEFAULT_PROFILE")) return *p;
    else return "default";
}

std::unique_ptr<S3::Auth> S3::Auth::create(
        const std::string s,
        const std::string profile)
{
    const json config(s.size() ? json::parse(s) : json());

    // Try explicit JSON configuration first.
    if (
            !config.is_null() &&
            config.count("access") &&
            (config.count("secret") || config.count("hidden")))
    {
        return makeUnique<Auth>(
                config["access"].get<std::string>(),
                config.count("secret") ?
                    config["secret"].get<std::string>() :
                    config["hidden"].get<std::string>(),
                config.value("token", ""));
    }

    // Try environment settings next.
    {
        auto access(env("AWS_ACCESS_KEY_ID"));
        auto hidden(env("AWS_SECRET_ACCESS_KEY"));
        auto token(env("AWS_SESSION_TOKEN"));

        if (access && hidden)
        {
            return makeUnique<Auth>(*access, *hidden, token ? *token : "");
        }

        access = env("AMAZON_ACCESS_KEY_ID");
        hidden = env("AMAZON_SECRET_ACCESS_KEY");
        token = env("AMAZON_SESSION_TOKEN");

        if (access && hidden)
        {
            return makeUnique<Auth>(*access, *hidden, token ? *token : "");
        }
    }

    const std::string credPath(
            env("AWS_CREDENTIAL_FILE") ?
                *env("AWS_CREDENTIAL_FILE") : "~/.aws/credentials");

    // Finally, try reading credentials file.
    drivers::Fs fsDriver;
    if (std::unique_ptr<std::string> c = fsDriver.tryGet(credPath))
    {
        const std::string accessKey("aws_access_key_id");
        const std::string hiddenKey("aws_secret_access_key");
        const ini::Contents creds(ini::parse(*c));
        if (creds.count(profile))
        {
            const auto section(creds.at(profile));
            if (section.count(accessKey) && section.count(hiddenKey))
            {
                const auto access(section.at(accessKey));
                const auto hidden(section.at(hiddenKey));
                return makeUnique<Auth>(access, hidden);
            }
        }
    }

#ifdef ARBITER_CURL
    // Nothing found in the environment or on the filesystem.  However we may
    // be running in an EC2 instance with an instance profile set up.
    //
    // By default we won't search for this since we don't really want to make
    // an HTTP request on every Arbiter construction - but if we're allowed,
    // see if we can request an instance profile configuration.
    if (
            (!config.is_null() &&
                config.value("allowInstanceProfile", false)) ||
            env("AWS_ALLOW_INSTANCE_PROFILE"))
    {
        http::Pool pool;
        drivers::Http httpDriver(pool);

        if (const auto iamRole = httpDriver.tryGet(credBase))
        {
            return makeUnique<Auth>(*iamRole);
        }
    }
#endif

    return std::unique_ptr<Auth>();
}

S3::Config::Config(const std::string s, const std::string profile)
    : m_region(extractRegion(s, profile))
    , m_baseUrl(extractBaseUrl(s, m_region))
{
    const json c(s.size() ? json::parse(s) : json());
    if (c.is_null()) return;

    m_precheck = c.value("precheck", false);

    if (c.value("sse", false) || env("AWS_SSE"))
    {
        m_baseHeaders["x-amz-server-side-encryption"] = "AES256";
    }

    if (c.value("requesterPays", false) || env("AWS_REQUESTER_PAYS"))
    {
        m_baseHeaders["x-amz-request-payer"] = "requester";
    }

    if (c.count("headers"))
    {
        const json& headers(c["headers"]);

        if (headers.is_object())
        {
            for (const auto& p : headers.items())
            {
                m_baseHeaders[p.key()] = p.value().get<std::string>();
            }
        }
        else
        {
            std::cout << "s3.headers expected to be object - skipping" <<
                std::endl;
        }
    }
}

std::string S3::Config::extractRegion(
        const std::string s,
        const std::string profile)
{
    const std::string configPath(
            env("AWS_CONFIG_FILE") ?
                *env("AWS_CONFIG_FILE") : "~/.aws/config");

    drivers::Fs fsDriver;

    const json c(s.size() ? json::parse(s) : json());

    if (!c.is_null() && c.count("region"))
    {
        return c.at("region").get<std::string>();
    }
    else if (auto p = env("AWS_REGION"))
    {
        return *p;
    }
    else if (auto p = env("AWS_DEFAULT_REGION"))
    {
        return *p;
    }
    else if (std::unique_ptr<std::string> c = fsDriver.tryGet(configPath))
    {
        const ini::Contents settings(ini::parse(*c));
        if (settings.count(profile))
        {
            const auto section(settings.at(profile));
            if (section.count("region")) return section.at("region");
        }
    }

    if (!c.is_null() && c.value("verbose", false))
    {
        std::cout << "Region not found - defaulting to us-east-1" << std::endl;
    }

    return "us-east-1";
}

std::string S3::Config::extractBaseUrl(
        const std::string s,
        const std::string region)
{
    const json c(s.size() ? json::parse(s) : json());

    if (!c.is_null() &&
            c.count("endpoint") &&
            c["endpoint"].get<std::string>().size())
    {
        const std::string path(c["endpoint"].get<std::string>());
        return path.back() == '/' ? path : path + '/';
    }

    std::string endpointsPath("~/.aws/endpoints.json");

    if (const auto e = env("AWS_ENDPOINTS_FILE"))
    {
        endpointsPath = *e;
    }
    else if (c.count("endpointsFile"))
    {
        endpointsPath = c["endpointsFile"].get<std::string>();
    }

    std::string dnsSuffix("amazonaws.com");

    drivers::Fs fsDriver;
    if (std::unique_ptr<std::string> e = fsDriver.tryGet(endpointsPath))
    {
        const json ep(json::parse(*e));

        for (const auto& partition : ep["partitions"])
        {
            if (partition.count("dnsSuffix"))
            {
                dnsSuffix = partition["dnsSuffix"].get<std::string>();
            }

            const auto& endpoints(
                    partition.at("services").at("s3").at("endpoints"));

            for (const auto& r : endpoints.items())
            {
                if (r.key() == region &&
                        endpoints.value("region", json::object())
                            .count("hostname"))
                {
                    return endpoints["region"]["hostname"].get<std::string>() +
                        '/';
                }
            }
        }
    }

    if (dnsSuffix.size() && dnsSuffix.back() != '/') dnsSuffix += '/';

    // https://docs.aws.amazon.com/general/latest/gr/rande.html#s3_region
    if (region == "us-east-1") return "s3." + dnsSuffix;
    else return "s3-" + region + "." + dnsSuffix;
}

S3::AuthFields S3::Auth::fields() const
{
#ifdef ARBITER_CURL
    if (m_role)
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        const Time now;
        if (!m_expiration || *m_expiration - now < reauthSeconds)
        {
            http::Pool pool;
            drivers::Http httpDriver(pool);

            const json creds(json::parse(httpDriver.get(credBase + *m_role)));
            m_access = creds.at("AccessKeyId").get<std::string>();
            m_hidden = creds.at("SecretAccessKey").get<std::string>();
            m_token = creds.at("Token").get<std::string>();
            m_expiration.reset(
                    new Time(
                        creds.at("Expiration").get<std::string>(),
                        arbiter::Time::iso8601));

            if (*m_expiration - now < reauthSeconds)
            {
                throw ArbiterError("Got invalid instance profile credentials");
            }
        }

        // If we're using an IAM role, make sure to create this before
        // releasing the lock.
        return S3::AuthFields(m_access, m_hidden, m_token);
    }
#endif

    return S3::AuthFields(m_access, m_hidden, m_token);
}

std::string S3::type() const
{
    if (m_profile == "default") return "s3";
    else return m_profile + "@s3";
}

std::unique_ptr<std::size_t> S3::tryGetSize(std::string rawPath) const
{
    std::unique_ptr<std::size_t> size;

    Headers headers(m_config->baseHeaders());
    headers.erase("x-amz-server-side-encryption");

    const Resource resource(m_config->baseUrl(), rawPath);
    const ApiV4 apiV4(
            "HEAD",
            m_config->region(),
            resource,
            m_auth->fields(),
            Query(),
            headers,
            empty);

    drivers::Http http(m_pool);
    Response res(http.internalHead(resource.url(), apiV4.headers()));

    if (res.ok() && res.headers().count("Content-Length"))
    {
        const std::string& str(res.headers().at("Content-Length"));
        size.reset(new std::size_t(std::stoul(str)));
    }

    return size;
}

bool S3::get(
        const std::string rawPath,
        std::vector<char>& data,
        const Headers userHeaders,
        const Query query) const
{
    Headers headers(m_config->baseHeaders());
    headers.erase("x-amz-server-side-encryption");
    headers.insert(userHeaders.begin(), userHeaders.end());

    std::unique_ptr<std::size_t> size(
            m_config->precheck() && !headers.count("Range") ?
                tryGetSize(rawPath) : nullptr);

    const Resource resource(m_config->baseUrl(), rawPath);
    const ApiV4 apiV4(
            "GET",
            m_config->region(),
            resource,
            m_auth->fields(),
            query,
            headers,
            empty);

    drivers::Http http(m_pool);
    Response res(
            http.internalGet(
                resource.url(),
                apiV4.headers(),
                apiV4.query(),
                size ? *size : 0));

    if (res.ok())
    {
        data = res.data();
        return true;
    }
    else
    {
        std::cout << res.code() << ": " << res.str() << std::endl;
        return false;
    }
}

void S3::put(
        const std::string rawPath,
        const std::vector<char>& data,
        const Headers userHeaders,
        const Query query) const
{
    const Resource resource(m_config->baseUrl(), rawPath);

    Headers headers(m_config->baseHeaders());
    headers.insert(userHeaders.begin(), userHeaders.end());

    if (getExtension(rawPath) == "json")
    {
        headers["Content-Type"] = "application/json";
    }

    const ApiV4 apiV4(
            "PUT",
            m_config->region(),
            resource,
            m_auth->fields(),
            query,
            headers,
            data);

    drivers::Http http(m_pool);
    Response res(
            http.internalPut(
                resource.url(),
                data,
                apiV4.headers(),
                apiV4.query()));

    if (!res.ok())
    {
        throw ArbiterError(
                "Couldn't S3 PUT to " + rawPath + ": " +
                std::string(res.data().data(), res.data().size()));
    }
}

void S3::copy(const std::string src, const std::string dst) const
{
    Headers headers;
    const Resource resource(m_config->baseUrl(), src);
    headers["x-amz-copy-source"] = resource.bucket() + '/' + resource.object();
    put(dst, std::vector<char>(), headers, Query());
}

std::vector<std::string> S3::glob(std::string path, bool verbose) const
{
    std::vector<std::string> results;
    path.pop_back();

    const bool recursive(path.back() == '*');
    if (recursive) path.pop_back();

    // https://docs.aws.amazon.com/AmazonS3/latest/API/RESTBucketGET.html
    const Resource resource(m_config->baseUrl(), path);
    const std::string& bucket(resource.bucket());
    const std::string& object(resource.object());

    Query query;

    if (object.size()) query["prefix"] = object;

    bool more(false);
    std::vector<char> data;

    do
    {
        if (verbose) std::cout << "." << std::flush;

        if (!get(resource.bucket() + "/", data, Headers(), query))
        {
            throw ArbiterError("Couldn't S3 GET " + resource.bucket());
        }

        data.push_back('\0');

        Xml::xml_document<> xml;

        try
        {
            xml.parse<0>(data.data());
        }
        catch (Xml::parse_error&)
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
                            results.push_back(
                                    type() + "://" + bucket + "/" + key);
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

S3::ApiV4::ApiV4(
        const std::string verb,
        const std::string& region,
        const Resource& resource,
        const S3::AuthFields authFields,
        const Query& query,
        const Headers& headers,
        const std::vector<char>& data)
    : m_authFields(authFields)
    , m_region(region)
    , m_time()
    , m_headers(headers)
    , m_query(query)
    , m_signedHeadersString()
{
    m_headers["Host"] = resource.host();
    m_headers["X-Amz-Date"] = m_time.str(Time::iso8601NoSeparators);
    if (m_authFields.token().size())
    {
        m_headers["X-Amz-Security-Token"] = m_authFields.token();
    }
    m_headers["X-Amz-Content-Sha256"] =
            crypto::encodeAsHex(crypto::sha256(data));

    if (verb == "PUT" || verb == "POST")
    {
        if (!m_headers.count("Content-Type"))
        {
            m_headers["Content-Type"] = "application/octet-stream";
        }
        m_headers.erase("Transfer-Encoding");
        m_headers.erase("Expect");
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

std::string S3::ApiV4::buildCanonicalRequest(
        const std::string verb,
        const Resource& resource,
        const Query& query,
        const std::vector<char>& data) const
{
    const std::string canonicalUri("/" + resource.object());

    auto canonicalizeQuery([](const std::string& s, const Query::value_type& q)
    {
        const std::string keyVal(
                sanitize(q.first, "") + '=' +
                sanitize(q.second, ""));

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

std::string S3::ApiV4::buildStringToSign(
        const std::string& canonicalRequest) const
{
    return
        line("AWS4-HMAC-SHA256") +
        line(m_time.str(Time::iso8601NoSeparators)) +
        line(m_time.str(Time::dateNoSeparators) +
                "/" + m_region + "/s3/aws4_request") +
        crypto::encodeAsHex(crypto::sha256(canonicalRequest));
}

std::string S3::ApiV4::calculateSignature(
        const std::string& stringToSign) const
{
    const std::string kDate(
            crypto::hmacSha256(
                "AWS4" + m_authFields.hidden(),
                m_time.str(Time::dateNoSeparators)));

    const std::string kRegion(crypto::hmacSha256(kDate, m_region));
    const std::string kService(crypto::hmacSha256(kRegion, "s3"));
    const std::string kSigning(
            crypto::hmacSha256(kService, "aws4_request"));

    return crypto::encodeAsHex(crypto::hmacSha256(kSigning, stringToSign));
}

std::string S3::ApiV4::getAuthHeader(
        const std::string& signedHeadersString,
        const std::string& signature) const
{
    return
        std::string("AWS4-HMAC-SHA256 ") +
        "Credential=" + m_authFields.access() + '/' +
            m_time.str(Time::dateNoSeparators) + "/" +
            m_region + "/s3/aws4_request, " +
        "SignedHeaders=" + signedHeadersString + ", " +
        "Signature=" + signature;
}

S3::Resource::Resource(std::string base, std::string fullPath)
    : m_baseUrl(base)
    , m_bucket()
    , m_object()
    , m_virtualHosted(true)
{
    fullPath = sanitize(fullPath);
    const std::size_t split(fullPath.find("/"));

    m_bucket = fullPath.substr(0, split);
    if (split != std::string::npos) m_object = fullPath.substr(split + 1);

    // We would prefer to use virtual-hosted URLs all the time since path-style
    // URLs are being deprecated in 2020.  We also want to use HTTPS all the
    // time, which is required for KMS-managed server-side encryption.  However,
    // these two desires are incompatible if the bucket name contains dots,
    // because the SSL cert will not allow virtual-hosted paths to be accessed
    // over HTTPS.  So we'll fall back to path-style requests for buckets
    // containing dots.  A fix for this should be announced by September 30,
    // 2020, at which point maybe we can use virtual-hosted paths all the time.

    // Deprecation plan for path-style URLs:
    // https://aws.amazon.com/blogs/aws/amazon-s3-path-deprecation-plan-the-rest-of-the-story/

    // Dots in bucket name limitation with virtual-hosting over HTTPS:
    // https://docs.aws.amazon.com/AmazonS3/latest/dev/VirtualHosting.html#VirtualHostingLimitations
    m_virtualHosted = m_bucket.find_first_of('.') == std::string::npos;
}

std::string S3::Resource::baseUrl() const
{
    return m_baseUrl;
}

std::string S3::Resource::bucket() const
{
    return m_virtualHosted ? m_bucket : "";
}

std::string S3::Resource::url() const
{
    if (m_virtualHosted)
    {
        return "https://" + m_bucket + "." + m_baseUrl + m_object;
    }
    else
    {
        return "https://" + m_baseUrl + m_bucket + "/" + m_object;
    }
}

std::string S3::Resource::object() const
{
    // We can't use virtual-host style paths if the bucket contains dots.
    if (m_virtualHosted) return m_object;
    else return m_bucket + "/" + m_object;
}

std::string S3::Resource::host() const
{
    if (m_virtualHosted)
    {
        // Pop slash.
        return m_bucket + "." + m_baseUrl.substr(0, m_baseUrl.size() - 1);
    }
    else
    {
        return m_baseUrl.substr(0, m_baseUrl.size() - 1);
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
// Beginning of content of file: arbiter/drivers/google.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/drivers/google.hpp>
#endif

#include <vector>

#ifdef ARBITER_OPENSSL
#include <openssl/evp.h>
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <openssl/sha.h>

// See: https://www.openssl.org/docs/manmaster/man3/OPENSSL_VERSION_NUMBER.html
#   if OPENSSL_VERSION_NUMBER >= 0x010100000
#   define ARBITER_OPENSSL_ATLEAST_1_1
#   endif

#endif

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/arbiter.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/util/json.hpp>
#include <arbiter/util/transforms.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using namespace internal;

namespace
{
    std::mutex sslMutex;

    const char baseGoogleUrl[] = "www.googleapis.com/storage/v1/";
    const char uploadUrl[] = "www.googleapis.com/upload/storage/v1/";
    const http::Query altMediaQuery{ { "alt", "media" } };

    class GResource
    {
    public:
        GResource(std::string path)
        {
            const std::size_t split(path.find("/"));
            m_bucket = path.substr(0, split) + "/";
            if (split != std::string::npos) m_object = path.substr(split + 1);
        }

        const std::string& bucket() const { return m_bucket; }
        const std::string& object() const { return m_object; }
        static const char exclusions[];
        std::string endpoint() const
        {
            // https://cloud.google.com/storage/docs/json_api/v1/
            return
                std::string(baseGoogleUrl) + "b/" + bucket() +
                "o/" + http::sanitize(object(), exclusions);
        }

        std::string uploadEndpoint() const
        {
            return std::string(uploadUrl) + "b/" + bucket() + "o";
        }

        std::string listEndpoint() const
        {
            return std::string(baseGoogleUrl) + "b/" + bucket() + "o";
        }

    private:
        std::string m_bucket;
        std::string m_object;

    };

    // https://cloud.google.com/storage/docs/json_api/#encoding
    const char GResource::exclusions[] = "!$&'()*+,;=:@";

} // unnamed namespace

namespace drivers
{

Google::Google(http::Pool& pool, std::unique_ptr<Auth> auth)
    : Https(pool)
    , m_auth(std::move(auth))
{ }

std::unique_ptr<Google> Google::create(http::Pool& pool, const std::string s)
{
    if (auto auth = Auth::create(s))
    {
        return makeUnique<Google>(pool, std::move(auth));
    }

    return std::unique_ptr<Google>();
}

std::unique_ptr<std::size_t> Google::tryGetSize(const std::string path) const
{
    http::Headers headers(m_auth->headers());
    const GResource resource(path);

    drivers::Https https(m_pool);
    const auto res(
            https.internalHead(resource.endpoint(), headers, altMediaQuery));

    if (res.ok())
    {
        if (res.headers().count("Content-Length"))
        {
            const auto& s(res.headers().at("Content-Length"));
            return makeUnique<std::size_t>(std::stoull(s));
        }
        else if (res.headers().count("content-length"))
        {
            const auto& s(res.headers().at("content-length"));
            return makeUnique<std::size_t>(std::stoull(s));
        }
    }

    return std::unique_ptr<std::size_t>();
}

bool Google::get(
        const std::string path,
        std::vector<char>& data,
        const http::Headers userHeaders,
        const http::Query query) const
{
    http::Headers headers(m_auth->headers());
    headers.insert(userHeaders.begin(), userHeaders.end());
    const GResource resource(path);

    drivers::Https https(m_pool);
    const auto res(
            https.internalGet(resource.endpoint(), headers, altMediaQuery));

    if (res.ok())
    {
        data = res.data();
        return true;
    }
    else
    {
        std::cout <<
            "Failed get - " << res.code() << ": " << res.str() << std::endl;
        return false;
    }
}

void Google::put(
        const std::string path,
        const std::vector<char>& data,
        const http::Headers userHeaders,
        const http::Query userQuery) const
{
    const GResource resource(path);
    const std::string url(resource.uploadEndpoint());

    http::Headers headers(m_auth->headers());
    headers["Expect"] = "";
    headers.insert(userHeaders.begin(), userHeaders.end());

    http::Query query(userQuery);
    query["uploadType"] = "media";
    query["name"] = http::sanitize(resource.object(), GResource::exclusions);

    drivers::Https https(m_pool);
    const auto res(https.internalPost(url, data, headers, query));
}

std::vector<std::string> Google::glob(std::string path, bool verbose) const
{
    std::vector<std::string> results;

    path.pop_back();
    const bool recursive(path.back() == '*');
    if (recursive) path.pop_back();

    const GResource resource(path);
    const std::string url(resource.listEndpoint());
    std::string pageToken;

    drivers::Https https(m_pool);
    http::Query query;

    // When the delimiter is set to "/", then the response will contain a
    // "prefixes" key in addition to the "items" key.  The "prefixes" key will
    // contain the directories found, which we will ignore.
    if (!recursive) query["delimiter"] = "/";
    if (resource.object().size()) query["prefix"] = resource.object();

    do
    {
        if (pageToken.size()) query["pageToken"] = pageToken;

        const auto res(https.internalGet(url, m_auth->headers(), query));

        if (!res.ok())
        {
            throw ArbiterError(std::to_string(res.code()) + ": " + res.str());
        }

        const json j(json::parse(res.str()));
        for (const json& item : j.at("items"))
        {
            results.push_back(
                    type() + "://" +
                    resource.bucket() + item.at("name").get<std::string>());
        }

        pageToken = j.value("nextPageToken", "");
    } while (pageToken.size());

    return results;
}

///////////////////////////////////////////////////////////////////////////////

std::unique_ptr<Google::Auth> Google::Auth::create(const std::string s)
{
    const json j(json::parse(s));
    if (auto path = env("GOOGLE_APPLICATION_CREDENTIALS"))
    {
        if (const auto file = drivers::Fs().tryGet(*path))
        {
            try
            {
                return makeUnique<Auth>(*file);
            }
            catch (const ArbiterError& e)
            {
                std::cout << e.what() << std::endl;
                return std::unique_ptr<Auth>();
            }
        }
    }
    else if (j.is_string())
    {
        const auto path(j.get<std::string>());
        if (const auto file = drivers::Fs().tryGet(path))
        {
            return makeUnique<Auth>(*file);
        }
    }
    else if (j.is_object())
    {
        return makeUnique<Auth>(s);
    }

    return std::unique_ptr<Auth>();
}

Google::Auth::Auth(const std::string s)
    : m_clientEmail(json::parse(s).at("client_email").get<std::string>())
    , m_privateKey(json::parse(s).at("private_key").get<std::string>())
{
    maybeRefresh();
}

http::Headers Google::Auth::headers() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    maybeRefresh();
    return m_headers;
}

void Google::Auth::maybeRefresh() const
{
    using namespace crypto;

    const auto now(Time().asUnix());
    if (m_expiration - now > 120) return;   // Refresh when under 2 mins left.

    // https://developers.google.com/identity/protocols/OAuth2ServiceAccount
    const json h { { "alg", "RS256" }, { "typ", "JWT" } };
    const json c {
        { "iss", m_clientEmail },
        { "scope", "https://www.googleapis.com/auth/devstorage.read_write" },
        { "aud", "https://www.googleapis.com/oauth2/v4/token" },
        { "iat", now },
        { "exp", now + 3600 }
    };

    const std::string header(encodeBase64(h.dump()));
    const std::string claims(encodeBase64(c.dump()));

    const std::string key(m_privateKey);
    const std::string signature(
            http::sanitize(encodeBase64(sign(header + '.' + claims, key))));

    const std::string assertion(header + '.' + claims + '.' + signature);
    const std::string sbody =
        "grant_type=urn%3Aietf%3Aparams%3Aoauth%3Agrant-type%3Ajwt-bearer&"
        "assertion=" + assertion;
    const std::vector<char> body(sbody.begin(), sbody.end());

    const http::Headers headers { { "Expect", "" } };
    const std::string tokenRequestUrl("www.googleapis.com/oauth2/v4/token");

    http::Pool pool;
    drivers::Https https(pool);
    const auto res(https.internalPost(tokenRequestUrl, body, headers));

    if (!res.ok())
    {
        throw ArbiterError(
                "Failed to get token for Google authentication, "
                "request came back with response: " + res.str());
    }

    const json token(json::parse(res.str()));
    m_headers["Authorization"] =
        "Bearer " + token.at("access_token").get<std::string>();
    m_expiration = now + token.at("expires_in").get<int64_t>();
}

std::string Google::Auth::sign(
        const std::string data,
        const std::string pkey) const
{
    std::string signature;

#ifdef ARBITER_OPENSSL
    std::lock_guard<std::mutex> lock(sslMutex);

    auto loadKey([](std::string s, bool isPublic)->EVP_PKEY*
    {
        // BIO_new_mem_buf needs non-const char*, so use a vector.
        std::vector<char> vec(s.data(), s.data() + s.size());

        EVP_PKEY* key(nullptr);
        if (BIO* bio = BIO_new_mem_buf(vec.data(), vec.size()))
        {
            if (isPublic)
            {
                key = PEM_read_bio_PUBKEY(bio, &key, nullptr, nullptr);
            }
            else
            {
                key = PEM_read_bio_PrivateKey(bio, &key, nullptr, nullptr);
            }

            BIO_free(bio);

            if (!key)
            {
                std::vector<char> err(256, 0);
                ERR_error_string(ERR_get_error(), err.data());
                throw ArbiterError(
                        std::string("Could not load key: ") + err.data());
            }
        }

        return key;
    });


    EVP_PKEY* key(loadKey(pkey, false));

#   ifdef ARBITER_OPENSSL_ATLEAST_1_1
    EVP_MD_CTX* ctx(EVP_MD_CTX_new());
#   else
    EVP_MD_CTX ctxAlloc;
    EVP_MD_CTX* ctx(&ctxAlloc);
#   endif
    EVP_MD_CTX_init(ctx);
    EVP_DigestSignInit(ctx, nullptr, EVP_sha256(), nullptr, key);

    if (EVP_DigestSignUpdate(ctx, data.data(), data.size()) == 1)
    {
        std::size_t size(0);
        if (EVP_DigestSignFinal(ctx, nullptr, &size) == 1)
        {
            std::vector<unsigned char> v(size, 0);
            if (EVP_DigestSignFinal(ctx, v.data(), &size) == 1)
            {
                signature.assign(reinterpret_cast<const char*>(v.data()), size);
            }
        }
    }

#   ifdef ARBITER_OPENSSL_ATLEAST_1_1
    EVP_MD_CTX_free(ctx);
#   else
    EVP_MD_CTX_cleanup(ctx);
#   endif

    if (signature.empty()) throw ArbiterError("Could not sign JWT");
    return signature;
#else
    throw ArbiterError("Cannot use google driver without OpenSSL");
#endif
}

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/google.cpp
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
#include <arbiter/util/json.hpp>
#endif



#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using namespace internal;

namespace
{
    const std::string baseDropboxUrl("https://content.dropboxapi.com/");
    const std::string getUrl(baseDropboxUrl + "2/files/download");
    const std::string putUrl(baseDropboxUrl + "2/files/upload");

    const std::string listUrl("https://api.dropboxapi.com/2/files/list_folder");
    const std::string metaUrl("https://api.dropboxapi.com/2/files/get_metadata");
    const std::string continueListUrl(listUrl + "/continue");

    const auto ins([](unsigned char lhs, unsigned char rhs)
    {
        return std::tolower(lhs) == std::tolower(rhs);
    });

    const std::string dirTag("folder");
    const std::string fileTag("file");
}

namespace drivers
{

using namespace http;

Dropbox::Dropbox(Pool& pool, const Dropbox::Auth& auth)
    : Http(pool)
    , m_auth(auth)
{ }

std::unique_ptr<Dropbox> Dropbox::create(Pool& pool, const std::string s)
{
    const json j(json::parse(s));
    if (!j.is_null())
    {
        if (j.is_object() && j.count("token"))
        {
            return makeUnique<Dropbox>(
                    pool,
                    Auth(j.at("token").get<std::string>()));
        }
        else if (j.is_string())
        {
            return makeUnique<Dropbox>(pool, Auth(j.get<std::string>()));
        }
    }

    return std::unique_ptr<Dropbox>();
}

Headers Dropbox::httpGetHeaders() const
{
    Headers headers;

    headers["Authorization"] = "Bearer " + m_auth.token();

    headers["Transfer-Encoding"] = "";
    headers["Expect"] = "";

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

std::unique_ptr<std::size_t> Dropbox::tryGetSize(const std::string path) const
{
    std::unique_ptr<std::size_t> result;

    Headers headers(httpPostHeaders());

    json tx { { "path", "/" + path } };
    const std::string f(tx.dump());
    const std::vector<char> postData(f.begin(), f.end());

    Response res(Http::internalPost(metaUrl, postData, headers));

    if (res.ok())
    {
        const auto data(res.data());

        json rx(json::parse(std::string(data.data(), data.size())));
        if (rx.count("size"))
        {
            result = makeUnique<std::size_t>(rx.at("size").get<uint64_t>());
        }
    }

    return result;
}

bool Dropbox::get(
        const std::string path,
        std::vector<char>& data,
        const Headers userHeaders,
        const Query query) const
{
    Headers headers(httpGetHeaders());

    headers["Dropbox-API-Arg"] = json{{ "path", "/" + path }}.dump();
    headers.insert(userHeaders.begin(), userHeaders.end());

    const Response res(Http::internalGet(getUrl, headers, query));

    if (res.ok())
    {
        if (!userHeaders.count("Range"))
        {
            if (!res.headers().count("dropbox-api-result"))
            {
                std::cout << "No dropbox-api-result header found" << std::endl;
                return false;
            }

            json rx;
            try { rx = json::parse(res.headers().at("dropbox-api-result")); }
            catch (...) { std::cout << "Failed to parse result" << std::endl; }

            if (!rx.is_null())
            {
                if (!rx.count("size"))
                {
                    std::cout << "No size found in API result" << std::endl;
                    return false;
                }

                const std::size_t size(rx.at("size").get<std::size_t>());
                data = res.data();

                if (size == data.size()) return true;
                else
                {
                    std::cout <<
                            "Data size check failed - got " <<
                            size << " of " << data.size() << " bytes." <<
                            std::endl;
                }
            }
        }
        else
        {
            data = res.data();
            return true;
        }
    }
    else
    {
        const auto data(res.data());
        std::string message(data.data(), data.size());

        std::cout <<
                "Server response: " << res.code() << " - '" << message << "'" <<
                std::endl;
    }

    return false;
}

void Dropbox::put(
        const std::string path,
        const std::vector<char>& data,
        const Headers userHeaders,
        const Query query) const
{
    Headers headers(httpGetHeaders());
    headers["Dropbox-API-Arg"] = json{{ "path", "/" + path }}.dump();
    headers["Content-Type"] = "application/octet-stream";

    headers.insert(userHeaders.begin(), userHeaders.end());

    const Response res(Http::internalPost(putUrl, data, headers, query));

    if (!res.ok()) throw ArbiterError(res.str());
}

std::string Dropbox::continueFileInfo(std::string cursor) const
{
    Headers headers(httpPostHeaders());

    const std::string f(json{{ "cursor", cursor }}.dump());
    std::vector<char> postData(f.begin(), f.end());
    Response res(Http::internalPost(continueListUrl, postData, headers));

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

std::vector<std::string> Dropbox::glob(std::string path, bool verbose) const
{
    std::vector<std::string> results;

    path.pop_back();
    const bool recursive(path.back() == '*');
    if (recursive) path.pop_back();
    if (path.back() == '/') path.pop_back();

    auto listPath = [this, recursive](std::string path)->std::string
    {
        Headers headers(httpPostHeaders());

        const json request {
            { "path", "/" + path },
            { "recursive", recursive },
            { "include_media_info", false },
            { "include_deleted", false }
        };
        const std::string f(request.dump());
        std::vector<char> postData(f.begin(), f.end());

        // Can't fully qualify this protected method within the lambda due to a
        // GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61148
        Response res(internalPost(listUrl, postData, headers));

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

    auto processPath = [this, verbose, &results, &more, &cursor](std::string d)
    {
        if (d.empty()) return;
        if (verbose) std::cout << '.';

        const json j(json::parse(d));
        if (!j.count("entries"))
        {
            throw ArbiterError("Returned JSON from Dropbox was null");
        }
        const json& entries(j.at("entries"));
        if (!entries.is_array())
        {
            throw ArbiterError("Returned JSON from Dropbox was not an array");
        }

        more = j.value("has_more", false);
        cursor = j.value("cursor", "");

        for (std::size_t i(0); i < entries.size(); ++i)
        {
            const json& v(entries[i]);
            const std::string tag(v.value(".tag", ""));

            // Only insert files.
            if (std::equal(tag.begin(), tag.end(), fileTag.begin(), ins))
            {
                // Results already begin with a slash.
                results.push_back(
                        type() + ":/" + v.at("path_lower").get<std::string>());
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

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/dropbox.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/curl.cpp
// //////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cstring>
#include <ios>
#include <iostream>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/curl.hpp>
#include <arbiter/util/http.hpp>
#include <arbiter/util/util.hpp>
#include <arbiter/util/json.hpp>


#ifdef ARBITER_ZLIB
#include <arbiter/third/gzip/decompress.hpp>
#endif

#endif

#ifdef ARBITER_CURL
#include <curl/curl.h>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using namespace internal;

namespace http
{

namespace
{
#ifdef ARBITER_CURL
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
                (std::min)(
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
            http::Headers* out)
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

    std::size_t eatLogging(void *out, size_t size, size_t num, void *in)
    {
        return size * num;
    }

#else
    const std::string fail("Arbiter was built without curl");
#endif // ARBITER_CURL
} // unnamed namespace

Curl::Curl(const std::string s)
{
#ifdef ARBITER_CURL
    const json c(s.size() ? json::parse(s) : json::object());

    m_curl = curl_easy_init();

    // Configurable entries are:
    //      - timeout           (CURLOPT_LOW_SPEED_TIME)
    //      - followRedirect    (CURLOPT_FOLLOWLOCATION)
    //      - caBundle          (CURLOPT_CAPATH)
    //      - caInfo            (CURLOPT_CAINFO)
    //      - verifyPeer        (CURLOPT_SSL_VERIFYPEER)

    using Keys = std::vector<std::string>;
    auto find([](const Keys& keys)->std::unique_ptr<std::string>
    {
        for (const auto& key : keys)
        {
            if (auto e = env(key)) return makeUnique<std::string>(*e);
        }
        return std::unique_ptr<std::string>();
    });

    auto mk([](std::string s) { return makeUnique<std::string>(s); });

    if (!c.is_null())
    {
        m_verbose = c.value("verbose", false);
        const auto& h(c.value("http", json::object()));

        if (!h.is_null())
        {
            if (h.count("timeout"))
            {
                m_timeout = h["timeout"].get<long>();
            }

            if (h.count("followRedirect"))
            {
                m_followRedirect = h["followRedirect"].get<bool>();
            }

            if (h.count("caBundle"))
            {
                m_caPath = mk(h["caBundle"].get<std::string>());
            }
            else if (h.count("caPath"))
            {
                m_caPath = mk(h["caPath"].get<std::string>());
            }

            if (h.count("caInfo"))
            {
                m_caInfo = mk(h["caInfo"].get<std::string>());
            }

            if (h.count("verifyPeer"))
            {
                m_verifyPeer = h["verifyPeer"].get<bool>();
            }
        }
    }

    Keys verboseKeys{ "VERBOSE", "CURL_VERBOSE", "ARBITER_VERBOSE" };
    Keys timeoutKeys{ "CURL_TIMEOUT", "ARBITER_HTTP_TIMEOUT" };
    Keys redirKeys{
        "CURL_FOLLOWLOCATION",
        "CURL_FOLLOW_LOCATION",
        "ARBITER_FOLLOW_LOCATION"
        "ARBITER_FOLLOW_REDIRECT"
    };
    Keys verifyKeys{
        "CURL_SSL_VERIFYPEER",
        "CURL_VERIFY_PEER",
        "ARBITER_VERIFY_PEER"
    };
    Keys caPathKeys{ "CURL_CA_PATH", "CURL_CA_BUNDLE", "ARBITER_CA_PATH" };
    Keys caInfoKeys{ "CURL_CAINFO", "CURL_CA_INFO", "ARBITER_CA_INFO" };

    if (auto v = find(verboseKeys)) m_verbose = !!std::stol(*v);
    if (auto v = find(timeoutKeys)) m_timeout = std::stol(*v);
    if (auto v = find(redirKeys)) m_followRedirect = !!std::stol(*v);
    if (auto v = find(verifyKeys)) m_verifyPeer = !!std::stol(*v);
    if (auto v = find(caPathKeys)) m_caPath = mk(*v);
    if (auto v = find(caInfoKeys)) m_caInfo = mk(*v);

    static bool logged(false);
    if (m_verbose && !logged)
    {
        logged = true;
        std::cout << "Curl config:" << std::boolalpha <<
            "\n\ttimeout: " << m_timeout << "s" <<
            "\n\tfollowRedirect: " << m_followRedirect <<
            "\n\tverifyPeer: " << m_verifyPeer <<
            "\n\tcaBundle: " << (m_caPath ? *m_caPath : "(default)") <<
            "\n\tcaInfo: " << (m_caInfo ? *m_caInfo : "(default)") <<
            std::endl;
    }
#endif
}

Curl::~Curl()
{
#ifdef ARBITER_CURL
    curl_easy_cleanup(m_curl);
    curl_slist_free_all(m_headers);
    m_headers = nullptr;
#endif
}

void Curl::init(
        const std::string rawPath,
        const Headers& headers,
        const Query& query)
{
#ifdef ARBITER_CURL
    // Reset our curl instance and header list.
    curl_slist_free_all(m_headers);
    m_headers = nullptr;

    // Set path.
    const std::string path(rawPath + buildQueryString(query));
    curl_easy_setopt(m_curl, CURLOPT_URL, path.c_str());

    // Needed for multithreaded Curl usage.
    curl_easy_setopt(m_curl, CURLOPT_NOSIGNAL, 1L);

    // Substantially faster DNS lookups without IPv6.
    curl_easy_setopt(m_curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);

    // Don't wait forever.  Use the low-speed options instead of the timeout
    // option to make the timeout a sliding window instead of an absolute.
    curl_easy_setopt(m_curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
    curl_easy_setopt(m_curl, CURLOPT_LOW_SPEED_TIME, m_timeout);

    curl_easy_setopt(m_curl, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
    curl_easy_setopt(m_curl, CURLOPT_ACCEPTTIMEOUT_MS, 2000L);

    auto toLong([](bool b) { return b ? 1L : 0L; });

    // Configuration options.
    curl_easy_setopt(m_curl, CURLOPT_VERBOSE, toLong(m_verbose));
    curl_easy_setopt(m_curl, CURLOPT_FOLLOWLOCATION, toLong(m_followRedirect));
    curl_easy_setopt(m_curl, CURLOPT_SSL_VERIFYPEER, toLong(m_verifyPeer));
    if (m_caPath) curl_easy_setopt(m_curl, CURLOPT_CAPATH, m_caPath->c_str());
    if (m_caInfo) curl_easy_setopt(m_curl, CURLOPT_CAINFO, m_caInfo->c_str());

    // Insert supplied headers.
    for (const auto& h : headers)
    {
        m_headers = curl_slist_append(
                m_headers,
                (h.first + ": " + h.second).c_str());
    }
#else
    throw ArbiterError(fail);
#endif
}

int Curl::perform()
{
#ifdef ARBITER_CURL
    long httpCode(0);

    const auto code(curl_easy_perform(m_curl));
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &httpCode);
    curl_easy_reset(m_curl);

    if (code != CURLE_OK) httpCode = 500;

    return httpCode;
#else
    throw ArbiterError(fail);
#endif
}

Response Curl::get(
        std::string path,
        Headers headers,
        Query query,
        const std::size_t reserve)
{
#ifdef ARBITER_CURL
    std::vector<char> data;

    if (reserve) data.reserve(reserve);

    init(path, headers, query);

    // Register callback function and data pointer to consume the result.
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, getCb);
    curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &data);

    // Insert all headers into the request.
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, m_headers);

    // Set up callback and data pointer for received headers.
    Headers receivedHeaders;
    curl_easy_setopt(m_curl, CURLOPT_HEADERFUNCTION, headerCb);
    curl_easy_setopt(m_curl, CURLOPT_HEADERDATA, &receivedHeaders);

    // Run the command.
    const int httpCode(perform());

    for (auto& h : receivedHeaders)
    {
        std::string& v(h.second);
        while (v.size() && v.front() == ' ') v = v.substr(1);
        while (v.size() && v.back() == ' ') v.pop_back();
    }

    if (receivedHeaders["Content-Encoding"] == "gzip")
    {
#ifdef ARBITER_ZLIB
        std::string s(gzip::decompress(data.data(), data.size()));
        data.assign(s.begin(), s.end());
#else
        throw ArbiterError("Cannot decompress zlib");
#endif
    }

    return Response(httpCode, data, receivedHeaders);
#else
    throw ArbiterError(fail);
#endif
}

Response Curl::head(std::string path, Headers headers, Query query)
{
#ifdef ARBITER_CURL
    std::vector<char> data;

    init(path, headers, query);

    // Register callback function and data pointer to consume the result.
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
    const int httpCode(perform());
    return Response(httpCode, data, receivedHeaders);
#else
    throw ArbiterError(fail);
#endif
}

Response Curl::put(
        std::string path,
        const std::vector<char>& data,
        Headers headers,
        Query query)
{
#ifdef ARBITER_CURL
    init(path, headers, query);

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
    const int httpCode(perform());
    return Response(httpCode);
#else
    throw ArbiterError(fail);
#endif
}

Response Curl::post(
        std::string path,
        const std::vector<char>& data,
        Headers headers,
        Query query)
{
#ifdef ARBITER_CURL
    init(path, headers, query);

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
    const int httpCode(perform());
    return Response(httpCode, writeData, receivedHeaders);
#else
    throw ArbiterError(fail);
#endif
}

} // namepace http
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/curl.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/http.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/http.hpp>
#include <arbiter/util/json.hpp>
#endif

#ifdef ARBITER_CURL
#include <curl/curl.h>
#endif

#include <cctype>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace http
{

std::string sanitize(const std::string path, const std::string excStr)
{
    static const std::set<char> unreserved = { '-', '.', '_', '~' };
    const std::set<char> exclusions(excStr.begin(), excStr.end());
    std::ostringstream result;
    result.fill('0');
    result << std::hex;

    for (const auto c : path)
    {
        if (std::isalnum(c) || unreserved.count(c) || exclusions.count(c))
        {
            result << c;
        }
        else
        {
            result << std::uppercase;
            result << '%' << std::setw(2) <<
                static_cast<int>(static_cast<uint8_t>(c));
            result << std::nouppercase;
        }
    }

    return result.str();
}

std::string buildQueryString(const Query& query)
{
    return std::accumulate(
            query.begin(),
            query.end(),
            std::string(),
            [](const std::string& out, const Query::value_type& keyVal)
            {
                const char sep(out.empty() ? '?' : '&');
                return out + sep + keyVal.first + '=' + keyVal.second;
            });
}

Resource::Resource(
        Pool& pool,
        Curl& curl,
        const std::size_t id,
        const std::size_t retry)
    : m_pool(pool)
    , m_curl(curl)
    , m_id(id)
    , m_retry(retry)
{ }

Resource::~Resource()
{
    m_pool.release(m_id);
}

Response Resource::get(
        const std::string path,
        const Headers headers,
        const Query query,
        const std::size_t reserve)
{
    return exec([this, path, headers, query, reserve]()->Response
    {
        return m_curl.get(path, headers, query, reserve);
    });
}

Response Resource::head(
        const std::string path,
        const Headers headers,
        const Query query)
{
    return exec([this, path, headers, query]()->Response
    {
        return m_curl.head(path, headers, query);
    });
}

Response Resource::put(
        std::string path,
        const std::vector<char>& data,
        const Headers headers,
        const Query query)
{
    return exec([this, path, &data, headers, query]()->Response
    {
        return m_curl.put(path, data, headers, query);
    });
}

Response Resource::post(
        std::string path,
        const std::vector<char>& data,
        const Headers headers,
        const Query query)
{
    return exec([this, path, &data, headers, query]()->Response
    {
        return m_curl.post(path, data, headers, query);
    });
}

Response Resource::exec(std::function<Response()> f)
{
    Response res;
    std::size_t tries(0);

    do
    {
        res = f();
    }
    while (res.serverError() && tries++ < m_retry);

    return res;
}

///////////////////////////////////////////////////////////////////////////////

Pool::Pool(
        const std::size_t concurrent,
        const std::size_t retry,
        const std::string s)
    : m_curls(concurrent)
    , m_available(concurrent)
    , m_retry(retry)
    , m_mutex()
    , m_cv()
{
#ifdef ARBITER_CURL
    curl_global_init(CURL_GLOBAL_ALL);

    const json config(s.size() ? json::parse(s) : json::object());

    for (std::size_t i(0); i < concurrent; ++i)
    {
        m_available[i] = i;
        m_curls[i].reset(new Curl(config.dump()));
    }
#endif
}

Pool::~Pool() { }

Resource Pool::acquire()
{
    if (m_curls.empty())
    {
        throw std::runtime_error("Cannot acquire from empty pool");
    }

    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this]()->bool { return !m_available.empty(); });

    const std::size_t id(m_available.back());
    Curl& curl(*m_curls[id]);

    m_available.pop_back();

    return Resource(*this, curl, id, m_retry);
}

void Pool::release(const std::size_t id)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_available.push_back(id);
    lock.unlock();

    m_cv.notify_one();
}

} // namepace http
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/http.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/ini.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/ini.hpp>
#endif

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/util.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace ini
{

Contents parse(const std::string& s)
{
    Contents contents;

    Section section;

    const std::vector<std::string> lines;
    for (std::string line : split(s))
    {
        line = stripWhitespace(line);
        const std::size_t semiPos(line.find_first_of(';'));
        const std::size_t hashPos(line.find_first_of('#'));
        line = line.substr(0, std::min(semiPos, hashPos));

        if (line.size())
        {
            if (line.front() == '[' && line.back() == ']')
            {
                section = line.substr(1, line.size() - 2);
            }
            else
            {
                const std::size_t equals(line.find_first_of('='));
                if (equals != std::string::npos)
                {
                    const Key key(line.substr(0, equals));
                    const Val val(line.substr(equals + 1));
                    contents[section][key] = val;
                }
            }
        }
    }

    return contents;
}

} // namespace ini
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/ini.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/md5.cpp
// //////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <cstdlib>
#include <cstring>
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
        std::memset(ctx->data, 0, 56);
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
#include <cstring>
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
        std::memset(ctx->data, 0, 56);
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

std::string encodeBase64(const std::vector<char>& data, const bool pad)
{
    std::vector<uint8_t> input;
    for (std::size_t i(0); i < data.size(); ++i)
    {
        char c(data[i]);
        input.push_back(*reinterpret_cast<uint8_t*>(&c));
    }

    const std::size_t fullSteps(data.size() / 3);
    const std::size_t remainder(data.size() % 3);

    while (input.size() % 3) input.push_back(0);
    uint8_t* pos(input.data());

    std::string output(fullSteps * 4, '_');
    std::size_t outIndex(0);

    const uint32_t mask(0x3F);

    for (std::size_t i(0); i < fullSteps; ++i)
    {
        uint32_t chunk((*pos) << 16 | *(pos + 1) << 8 | *(pos + 2));

        output[outIndex++] = base64Vals[(chunk >> 18) & mask];
        output[outIndex++] = base64Vals[(chunk >> 12) & mask];
        output[outIndex++] = base64Vals[(chunk >>  6) & mask];
        output[outIndex++] = base64Vals[chunk & mask];

        pos += 3;
    }

    if (remainder)
    {
        uint32_t chunk(*(pos) << 16 | *(pos + 1) << 8 | *(pos + 2));

        output.push_back(base64Vals[(chunk >> 18) & mask]);
        output.push_back(base64Vals[(chunk >> 12) & mask]);
        if (remainder == 2) output.push_back(base64Vals[(chunk >> 6) & mask]);

        if (pad)
        {
            while (output.size() % 4) output.push_back('=');
        }
    }

    return output;
}

std::string encodeBase64(const std::string& input, const bool pad)
{
    return encodeBase64(std::vector<char>(input.begin(), input.end()), pad);
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
// Beginning of content of file: arbiter/util/time.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/time.hpp>
#endif

#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/types.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace
{
    std::mutex mutex;

    int64_t utcOffsetSeconds(const std::time_t& now)
    {
        std::lock_guard<std::mutex> lock(mutex);
        std::tm utc(*std::gmtime(&now));
        std::tm loc(*std::localtime(&now));
        return (int64_t)std::difftime(std::mktime(&utc), std::mktime(&loc));
    }
}

const std::string Time::iso8601 = "%Y-%m-%dT%H:%M:%SZ";
const std::string Time::iso8601NoSeparators = "%Y%m%dT%H%M%SZ";
const std::string Time::dateNoSeparators = "%Y%m%d";

Time::Time()
{
    m_time = std::time(nullptr);
}

Time::Time(const std::string& s, const std::string& format)
{
    std::tm tm{};

#ifndef ARBITER_WINDOWS
    // We'd prefer to use get_time, but it has poor compiler support.
    if (!strptime(s.c_str(), format.c_str(), &tm))
    {
        throw ArbiterError("Failed to parse " + s + " as time: " + format);
    }
#else
    std::istringstream ss(s);
    ss >> std::get_time(&tm, format.c_str());
    if (ss.fail())
    {
        throw ArbiterError("Failed to parse " + s + " as time: " + format);
    }
#endif
    std::time_t time = std::mktime(&tm)!=-1 ? std::mktime(&tm) :std::time(nullptr);
    const int64_t utcOffset(utcOffsetSeconds(time));

    if (utcOffset > std::numeric_limits<int>::max())
        throw ArbiterError("Can't convert offset time in seconds to tm type.");
    tm.tm_sec -= (int)utcOffset;
    m_time = std::mktime(&tm);
}

std::string Time::str(const std::string& format) const
{
    std::lock_guard<std::mutex> lock(mutex);
#ifndef ARBITER_WINDOWS
    // We'd prefer to use put_time, but it has poor compiler support.
    // We're already locked here for gmtime, so might as well make this static.
    static std::vector<char> s(256, 0);

    const std::size_t size =
        strftime(s.data(), s.size(), format.c_str(), std::gmtime(&m_time));

    return std::string(s.data(), s.data() + size);
#else
    std::ostringstream ss;
    ss << std::put_time(std::gmtime(&m_time), format.c_str());
    return ss.str();
#endif
}

int64_t Time::operator-(const Time& other) const
{
    return (int64_t)std::difftime(m_time, other.m_time);
}

int64_t Time::asUnix() const
{
    static const Time epoch("1970-01-01T00:00:00Z");
    return *this - epoch;
}

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/time.cpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/util.cpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/util.hpp>

#include <arbiter/arbiter.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <mutex>
#include <random>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace
{
    const std::string protocolDelimiter("://");

    std::mutex randomMutex;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned long long> distribution;
}

uint64_t randomNumber()
{
    std::lock_guard<std::mutex> lock(randomMutex);
    return distribution(gen);
}

std::string stripPostfixing(const std::string path)
{
    std::string stripped(path);

    for (std::size_t i(0); i < 2; ++i)
    {
        // Pop trailing asterisk, or double-trailing-asterisks for both non- and
        // recursive globs.
        if (!stripped.empty() && stripped.back() == '*') stripped.pop_back();
    }

    // Pop trailing slash, in which case the result is the innermost directory.
    while (!stripped.empty() && isSlash(stripped.back())) stripped.pop_back();

    return stripped;
}

std::string getBasename(const std::string fullPath)
{
    std::string result(stripProtocol(fullPath));

    const std::string stripped(stripPostfixing(stripProtocol(fullPath)));

    // Now do the real slash searching.
    std::size_t pos(stripped.rfind('/'));

    // Maybe windows
    if (pos == std::string::npos)
        pos = stripped.rfind('\\');

    if (pos != std::string::npos)
    {
        const std::string sub(stripped.substr(pos + 1));
        if (!sub.empty()) result = sub;
    }

    return result;
}

std::string getDirname(const std::string fullPath)
{
    std::string result("");

    const std::string stripped(stripPostfixing(stripProtocol(fullPath)));

    // Now do the real slash searching.
    const std::size_t pos(stripped.find_last_of("/\\"));

    if (pos != std::string::npos)
    {
        const std::string sub(stripped.substr(0, pos));
        result = sub;
    }

    const std::string protocol(getProtocol(fullPath));
    if (protocol != "file") result = protocol + "://" + result;

    return result;
}

std::unique_ptr<std::string> env(const std::string& var)
{
    std::unique_ptr<std::string> result;

#ifndef ARBITER_WINDOWS
    if (const char* c = getenv(var.c_str())) result.reset(new std::string(c));
#else
    char* c(nullptr);
    std::size_t size(0);

    if (!_dupenv_s(&c, &size, var.c_str()))
    {
        if (c)
        {
            result.reset(new std::string(c));
            free(c);
        }
    }
#endif

    return result;
}

std::vector<std::string> split(const std::string& in, const char delimiter)
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

std::string stripWhitespace(const std::string& in)
{
    std::string out(in);
    out.erase(
            std::remove_if(
                out.begin(),
                out.end(),
                [](char c) { return std::isspace(c); }),
            out.end());
    return out;
}

std::string getProtocol(const std::string path)
{
    std::string type("file");
    const std::size_t pos(path.find(protocolDelimiter));

    if (pos != std::string::npos)
    {
        type = path.substr(0, pos);
    }

    return type;
}

std::string stripProtocol(const std::string raw)
{
    std::string result(raw);
    const std::size_t pos(raw.find(protocolDelimiter));

    if (pos != std::string::npos)
    {
        result = raw.substr(pos + protocolDelimiter.size());
    }

    return result;
}

std::string getExtension(std::string path)
{
    path = getBasename(path);
    const std::size_t pos(path.find_last_of('.'));

    if (pos != std::string::npos) return path.substr(pos + 1);
    else return std::string();
}

std::string stripExtension(const std::string path)
{
    const std::size_t pos(path.find_last_of('.'));
    return path.substr(0, pos);
}

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/util.cpp
// //////////////////////////////////////////////////////////////////////





