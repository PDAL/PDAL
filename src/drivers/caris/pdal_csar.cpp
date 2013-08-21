/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

#include <pdal/drivers/caris/CloudReader.hpp>

#include "Utils.hpp"

#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif

#include <pdal/StageFactory.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/detail/utf8_codecvt_facet.hpp>

#ifdef _MSC_VER
#   define NOMINMAX
#   define WIN32_LEAN_AND_MEAN
#   include <Windows.h>
#   include <tchar.h>
#endif

#ifdef _MSC_VER
#   pragma warning(pop)
// decorated name length exceeded, name was truncated
#   pragma warning(disable : 4503)
#endif

namespace
{
//! CARIS CSAR file Point Cloud Reader
class FileCloudReader : public csar::CloudReader
{

public:
    SET_STAGE_NAME("drivers.csar.reader", "CARIS CSAR file Point Cloud Reader")

    static pdal::Reader* create(const pdal::Options& in_opts)
    {
        return new FileCloudReader(in_opts);
    }

    virtual const pdal::Options getDefaultOptions() const
    {
        pdal::Options options;
        options.add("filename", "", "file to read from");
        return options;
    }

protected:
    virtual std::string getURI() const
    {
        return csar::utils::systemPathToURI(m_filename);
    }

private:
    explicit FileCloudReader(const pdal::Options& options)
        : CloudReader(options)
        , m_filename(options.getValueOrThrow<std::string>("filename"))
    {
    }

private:
    std::string m_filename;
};

//! CARIS Database Point Cloud Reader
class DBCloudReader : public csar::CloudReader
{

public:
    SET_STAGE_NAME("drivers.csardb.reader", "CARIS Database Point Cloud Reader")

    static pdal::Reader* create(const pdal::Options& in_opts)
    {
        return new DBCloudReader(in_opts);
    }

    virtual const pdal::Options getDefaultOptions() const
    {
        pdal::Options options;
        options.add("connection", "", "Database connection string formated as: "
                    "username/password@hostname%dbname%boid");
        return options;
    }

protected:
    virtual std::string getURI() const
    {
        return "csardb:" + m_connectionString;
    }

private:
    explicit DBCloudReader(const pdal::Options& options)
        : CloudReader(options)
        , m_connectionString(options.getValueOrThrow<std::string>("connection"))
    {
    }

    std::string m_connectionString;
};

//************************************************************************
//! get the path to the support file folder
//************************************************************************
std::string
getSupportPath()
{
#ifdef _MSC_VER
    WCHAR moduleName[BUFSIZ] = L"";
    ::GetModuleFileNameW(::GetModuleHandleW(L"libpdal_plugin_reader_csar"), moduleName, BUFSIZ-1);
    boost::filesystem::path appDir = boost::filesystem::path(moduleName);
    appDir.remove_filename();
    appDir /= "support";
    return appDir.string(boost::filesystem::detail::utf8_codecvt_facet());
#else
    return "/usr/share/caris/support/";
#endif
}


} // namespace

extern "C"
void PDALRegister_reader_csar(void* in_factoryPtr)
{
    try
    {
        static bool initialized = false;
        if (!initialized)
        {
            initialized = true;

            // no point caching read-only sequential access
            const size_t cache_bytes = 0;
            int rc = caris_init(cache_bytes, getSupportPath().c_str());
            if (rc)
            {
                std::cerr
                        << FileCloudReader::s_getName()
                        << ": initialization failed: "
                        << caris_status_message(rc, false)
                        << std::endl;
            }
        }

        pdal::StageFactory& factory = *(pdal::StageFactory*)in_factoryPtr;
        factory.registerReader(FileCloudReader::s_getName(), &FileCloudReader::create);
        factory.registerReader(DBCloudReader::s_getName(), &DBCloudReader::create);
    }
    catch (std::exception const& e)
    {
        std::cerr
                << FileCloudReader::s_getName()
                << ": initialization failed: "
                << e.what()
                << std::endl;
    }
}
