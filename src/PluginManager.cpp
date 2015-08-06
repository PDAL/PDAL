/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

// The PluginManager was modeled very closely after the work of Gigi Sayfan in
// the Dr. Dobbs article:
// http://www.drdobbs.com/cpp/building-your-own-plugin-framework-part/206503957
// The original work was released under the Apache License v2.

#include <pdal/PluginManager.hpp>

#include <boost/filesystem.hpp>

#include <pdal/pdal_defines.h>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include "DynamicLibrary.h"

#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

namespace
{
#if defined(__APPLE__) && defined(__MACH__)
    const std::string dynamicLibraryExtension(".dylib");
#elif defined __linux__
    const std::string dynamicLibraryExtension(".so");
#elif defined _WIN32
    const std::string dynamicLibraryExtension(".dll");
#endif

    bool isValid(const PF_RegisterParams * params)
    {
        return (params && params->createFunc && params->destroyFunc);
    }

    bool pluginTypeValid(std::string pathname, PF_PluginType type)
    {
        bool valid(false);

        if (Utils::startsWith(pathname, "libpdal_plugin_kernel"))
        {
            if (type == PF_PluginType_Kernel)
                valid = true;
        }
        else if (Utils::startsWith(pathname, "libpdal_plugin_filter"))
        {
            if (type == PF_PluginType_Filter)
                valid = true;
        }
        else if (Utils::startsWith(pathname, "libpdal_plugin_reader"))
        {
            if (type == PF_PluginType_Reader)
                valid = true;
        }
        else if (Utils::startsWith(pathname, "libpdal_plugin_writer"))
        {
            if (type == PF_PluginType_Writer)
                valid = true;
        }

        return valid;
    }
}


bool PluginManager::registerObject(const std::string& objectType,
    const PF_RegisterParams* params)
{
    bool registered(false);

    if (isValid(params))
    {
        PluginManager& pm = PluginManager::getInstance();

        if (pm.m_version.major == params->version.major)
        {
            auto entry(std::make_pair(objectType, *params));
            registered = pm.m_exactMatchMap.insert(entry).second;
        }
    }

    return registered;
}


PluginManager& PluginManager::getInstance()
{
    static PluginManager instance;
    return instance;
}


void PluginManager::loadAll(PF_PluginType type)
{
    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // If we don't have a driver path, defaults are set.
    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH <<
            ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec = Utils::split2(pluginDir, ':');

    for (const auto& pluginPath : pluginPathVec)
    {
        loadAll(pluginPath, type);
    }
}


void PluginManager::loadAll(const std::string& pluginDirectory,
    PF_PluginType type)
{
    const bool pluginDirectoryValid =
            pluginDirectory.size() &&
                (FileUtils::fileExists(pluginDirectory) ||
                boost::filesystem::is_directory(pluginDirectory));

    if (pluginDirectoryValid)
    {
        boost::filesystem::directory_iterator dir(pluginDirectory), it, end;

        // Looks like directory_iterator doesn't support range-based for loop in
        // Boost v1.55. It fails Travis anyway, so I reverted it.
        for (it = dir; it != end; ++it)
        {
            boost::filesystem::path full_path = it->path();

            if (boost::filesystem::is_directory(full_path))
                continue;

            std::string ext = full_path.extension().string();
            if (ext != dynamicLibraryExtension)
                continue;

            loadByPath(full_path.string(), type);
        }
    }
}


bool PluginManager::initializePlugin(PF_InitFunc initFunc)
{
    bool initialized(false);

    PluginManager& pm = PluginManager::getInstance();

    if (PF_ExitFunc exitFunc = initFunc())
    {
        initialized = true;
        pm.m_exitFuncVec.push_back(exitFunc);
    }

    return initialized;
}


PluginManager::PluginManager()
{
    m_version.major = 1;
    m_version.minor = 0;
}


PluginManager::~PluginManager()
{
    if (!shutdown())
    {
        std::cerr << "Error destructing PluginManager" << std::endl;
    }
}


bool PluginManager::shutdown()
{
    bool success(true);

    for (auto const& func : m_exitFuncVec)
    {
        try
        {
            // Exit functions return 0 if successful.
            if ((*func)() != 0)
            {
                success = false;
            }
        }
        catch (...)
        {
            success = false;
        }
    }

    m_dynamicLibraryMap.clear();
    m_exactMatchMap.clear();
    m_exitFuncVec.clear();

    return success;
}

bool PluginManager::loadPlugin(const std::string& driverFileName)
{
    std::vector<std::string> driverPathVec;
    driverPathVec = Utils::split2(driverFileName, '.');

    boost::filesystem::path full_path(driverFileName);

    if (boost::filesystem::is_directory(full_path))
        return false;

    std::string ext = full_path.extension().string();
    if (ext != dynamicLibraryExtension)
        return false;
    std::string stem = full_path.stem().string();
    std::vector<std::string> driverNameVec;
    driverNameVec = Utils::split2(driverPathVec[0], '_');

    std::string ptype;
    if (driverNameVec.size() >=3)
        ptype = driverNameVec[2];

    PF_PluginType type;
    if (Utils::iequals(ptype, "reader"))
        type = PF_PluginType_Reader;
    else if (Utils::iequals(ptype,"kernel"))
        type = PF_PluginType_Kernel;
    else if (Utils::iequals(ptype, "filter"))
        type = PF_PluginType_Filter;
    else if (Utils::iequals(ptype, "writer"))
        type = PF_PluginType_Writer;
    else
        throw pdal_error("Unknown plugin type '" + ptype +"'");

    if (loadByPath(full_path.string(), type))
    {
        return true;
    }

    return false;
}

bool PluginManager::guessLoadByPath(const std::string& driverName)
{
    // parse the driver name into an expected pluginName, e.g.,
    // writers.las => libpdal_plugin_writer_las

    std::vector<std::string> driverNameVec;
    driverNameVec = Utils::split2(driverName, '.');

    std::string pluginName = "libpdal_plugin_" + driverNameVec[0] + "_" +
        driverNameVec[1];

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Default path below if not set.
    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH <<
            ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec = Utils::split2(pluginDir, ':');
    for (const auto& pluginPath : pluginPathVec)
    {
        boost::filesystem::path path(pluginPath);

        if (!FileUtils::fileExists(path.string()) ||
            !boost::filesystem::is_directory(path))
            continue;

        boost::filesystem::directory_iterator dir(path), it, end;
        // Looks like directory_iterator doesn't support range-based for loop in
        // Boost v1.55. It fails Travis anyway, so I reverted it.
        for (it = dir; it != end; ++it)
        {
            boost::filesystem::path full_path = it->path();

            if (boost::filesystem::is_directory(full_path))
                continue;

            std::string ext = full_path.extension().string();
            if (ext != dynamicLibraryExtension)
                continue;

            std::string stem = full_path.stem().string();
            std::string::size_type pos = stem.find_last_of('_');
            if (pos == std::string::npos || pos == stem.size() - 1 ||
                    stem.substr(pos + 1) != driverNameVec[1])
                continue;

            PF_PluginType type;
            if (driverNameVec[0] == "readers")
                type = PF_PluginType_Reader;
            else if (driverNameVec[0] == "kernels")
                type = PF_PluginType_Kernel;
            else if (driverNameVec[0] == "filters")
                type = PF_PluginType_Filter;
            else if (driverNameVec[0] == "writers")
                type = PF_PluginType_Writer;
            else
                type = PF_PluginType_Reader;

            if (loadByPath(full_path.string(), type))
            {
                return true;
            }
        }
    }

    return false;
}


bool PluginManager::loadByPath(const std::string& pluginPath,
    PF_PluginType type)
{
    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color

    bool loaded(false);

    boost::filesystem::path path(pluginPath);
    std::string pathname = Utils::tolower(path.filename().string());

    // If we are a valid type, and we're not yet already
    // loaded in the LibraryMap, load it.
    if (pluginTypeValid(pathname, type) &&
        m_dynamicLibraryMap.find(path.string()) == m_dynamicLibraryMap.end())
    {
        std::string errorString;
        auto completePath(boost::filesystem::complete(path).string());

        if (DynamicLibrary *d = loadLibrary(completePath, errorString))
        {
            if (PF_InitFunc initFunc =
                    (PF_InitFunc)(d->getSymbol("PF_initPlugin")))
            {
                loaded = initializePlugin(initFunc);
            }
        }
    }

    return loaded;
}

void *PluginManager::createObject(const std::string& objectType)
{
    void* obj(0);

    auto find([this, &objectType]()->bool
    {
        return m_exactMatchMap.count(objectType);
    });

    if (find() || (guessLoadByPath(objectType) && find()))
    {
        obj = m_exactMatchMap[objectType].createFunc();
    }

    return obj;
}


DynamicLibrary *PluginManager::loadLibrary(const std::string& path,
    std::string & errorString)
{
    DynamicLibrary *d = DynamicLibrary::load(path, errorString);

    if (d)
    {
        m_dynamicLibraryMap[boost::filesystem::complete(path).string()] =
            DynLibPtr(d);
    }

    return d;
}


const PluginManager::RegistrationMap& PluginManager::getRegistrationMap()
{
    return m_exactMatchMap;
}

} // namespace pdal
