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

#include <pdal/pdal_defines.h>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/Log.hpp>

#include "DynamicLibrary.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

namespace
{

static PluginManager s_instance;

#if defined(__APPLE__) && defined(__MACH__)
    const std::string dynamicLibraryExtension(".dylib");
#elif defined(__linux__) || defined(__FreeBSD_kernel__)
    const std::string dynamicLibraryExtension(".so");
#elif defined _WIN32
    const std::string dynamicLibraryExtension(".dll");
#endif


bool pluginTypeValid(std::string pathname, PF_PluginType type)
{
    return ((Utils::startsWith(pathname, "libpdal_plugin_kernel") &&
            type & PF_PluginType_Kernel) ||
        (Utils::startsWith(pathname, "libpdal_plugin_filter") &&
            type & PF_PluginType_Filter) ||
        (Utils::startsWith(pathname, "libpdal_plugin_reader") &&
            type & PF_PluginType_Reader) ||
        (Utils::startsWith(pathname, "libpdal_plugin_writer") &&
            type & PF_PluginType_Writer));
}

} // unnamed namespace;


bool PluginManager::registerObject(const std::string& name,
    const PF_RegisterParams *params)
{
    return s_instance.l_registerObject(name, params);
}


bool PluginManager::l_registerObject(const std::string& name,
    const PF_RegisterParams *params)
{
    if (params && params->createFunc && params->destroyFunc &&
        (m_version.major == params->version.major))
    {
        auto entry(std::make_pair(name, *params));

        std::lock_guard<std::mutex> lock(m_mutex);
        return m_plugins.insert(entry).second;
    }
    return false;
}


void PluginManager::loadAll(int type)
{
    s_instance.l_loadAll(type);
}


void PluginManager::l_loadAll(PF_PluginType type)
{
    std::string pluginDir;
    std::string driver_path("PDAL_DRIVER_PATH");
    Utils::getenv(driver_path, pluginDir);

    // If we don't have a driver path, defaults are set.
    if (pluginDir.empty())
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH <<
            ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec = Utils::split2(pluginDir, ':');

    for (const auto& pluginPath : pluginPathVec)
        loadAll(pluginPath, type);
}

StringList PluginManager::names(int typeMask)
{
    return s_instance.l_names(typeMask);
}


StringList PluginManager::l_names(int typeMask)
{
    StringList l;

    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto p : m_plugins)
        if (p.second.pluginType & typeMask)
            l.push_back(p.first);
    return l;
}

std::string PluginManager::link(const std::string& name)
{
    return s_instance.l_link(name);
}


std::string PluginManager::l_link(const std::string& name)
{
    std::string link;

    std::lock_guard<std::mutex> lock(m_mutex);
    auto ei = m_plugins.find(name);
    if (ei != m_plugins.end())
        link= ei->second.link;
    return link;
}


std::string PluginManager::description(const std::string& name)
{
    return s_instance.l_description(name);
}


std::string PluginManager::l_description(const std::string& name)
{
    std::string descrip;

    std::lock_guard<std::mutex> lock(m_mutex);
    auto ei = m_plugins.find(name);
    if (ei != m_plugins.end())
        descrip = ei->second.description;
    return descrip;
}


void PluginManager::loadAll(const std::string& pluginDirectory, int type)
{
    const bool pluginDirectoryValid = pluginDirectory.size() &&
        (FileUtils::fileExists(pluginDirectory) ||
            FileUtils::isDirectory(pluginDirectory));

    if (pluginDirectoryValid)
    {
        StringList files = FileUtils::directoryList(pluginDirectory);
        for (auto file : files)
        {
            if ((FileUtils::extension(file) == dynamicLibraryExtension) &&
                !FileUtils::isDirectory(file))
                loadByPath(file, type);
        }
    }
}


bool PluginManager::initializePlugin(PF_InitFunc initFunc)
{
    return s_instance.l_initializePlugin(initFunc);
}


bool PluginManager::l_initializePlugin(PF_InitFunc initFunc)
{
    if (PF_ExitFunc exitFunc = initFunc())
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_exitFuncVec.push_back(exitFunc);
        return true;
    }
    return false;
}


PluginManager::PluginManager()
{
    m_version.major = 1;
    m_version.minor = 0;
}


PluginManager::~PluginManager()
{
    if (!shutdown())
        Log("PDAL", "stderr").get(LogLevel::Error) <<
            "Error destroying PluginManager" << std::endl;
}


bool PluginManager::shutdown()
{
    std::lock_guard<std::mutex> lock(m_mutex);

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

    // This clears the handles on the dynamic libraries so that they won't
    // be closed with dlclose().  Depending on the order of dll unloading,
    // it's possible for a plugin library to be unloaded before this function
    // (which is usually in a dll) is called, which can raise an error on
    // some systems when dlclose() is called on a library for which the
    // reference count is already 0.  Since we're exiting anyway and all the
    // dlls will be closed, there is no need to call dlclose() on them
    // explicitly.
    for (auto l : m_dynamicLibraryMap)
        l.second->clear();

    m_dynamicLibraryMap.clear();
    m_plugins.clear();
    m_exitFuncVec.clear();

    return success;
}


bool PluginManager::loadPlugin(const std::string& driverFileName)
{
    return s_instance.l_loadPlugin(driverFileName);
}


bool PluginManager::l_loadPlugin(const std::string& driverFileName)
{
    std::vector<std::string> driverPathVec;
    driverPathVec = Utils::split2(driverFileName, '.');

    if ((FileUtils::extension(driverFileName) != dynamicLibraryExtension) ||
            FileUtils::isDirectory(driverFileName))
        return false;

    std::vector<std::string> driverNameVec;
    driverNameVec = Utils::split2(driverPathVec[0], '_');

    std::string ptype;
    if (driverNameVec.size() >= 3)
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
        throw pdal_error("Unknown plugin type '" + ptype + "'");

    return loadByPath(driverFileName, type);
}


bool PluginManager::guessLoadByPath(const std::string& driverName)
{
    // parse the driver name into an expected pluginName, e.g.,
    // writers.las => libpdal_plugin_writer_las

    std::vector<std::string> driverNameVec;
    driverNameVec = Utils::split2(driverName, '.');

    std::string pluginName = "libpdal_plugin_" + driverNameVec[0] + "_" +
        driverNameVec[1];

    std::string pluginDir;
    std::string driver_path("PDAL_DRIVER_PATH");
    Utils::getenv(driver_path, pluginDir);

    // Default path below if not set.
    if (pluginDir.empty())
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH <<
            ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    Log("PDAL", "stderr").get(LogLevel::Debug) <<
        "Plugin search path '" << pluginDir << "'" << std::endl;

    std::vector<std::string> pluginPathVec = Utils::split2(pluginDir, ':');
    for (const auto& pluginPath : pluginPathVec)
    {
        if (!FileUtils::fileExists(pluginPath) ||
            !FileUtils::isDirectory(pluginPath))
            continue;

        StringList files = FileUtils::directoryList(pluginPath);
        for (auto file : files)
        {
            if ((FileUtils::extension(file) != dynamicLibraryExtension) ||
                FileUtils::isDirectory(file))
                continue;

            std::string stem = FileUtils::stem(file);
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

            if (loadByPath(file, type))
                return true;
        }
    }

    return false;
}


bool PluginManager::loadByPath(const std::string& pluginPath, int type)
{
    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color

    bool loaded(false);

    std::string filename = Utils::tolower(FileUtils::getFilename(pluginPath));

    // If we are a valid type, and we're not yet already
    // loaded in the LibraryMap, load it.

    Log log("PDAL", "stderr");

    if (pluginTypeValid(filename, type) && !libraryLoaded(pluginPath))
    {
        std::string errorString;
        auto completePath(FileUtils::toAbsolutePath(pluginPath));

        log.get(LogLevel::Debug) << "Attempting to load plugin '" <<
            completePath << "'." << std::endl;

        if (DynamicLibrary *d = loadLibrary(completePath, errorString))
        {
            log.get(LogLevel::Debug) << "Loaded plugin '" << completePath <<
                "'." << std::endl;
            if (PF_InitFunc initFunc =
                    (PF_InitFunc)(d->getSymbol("PF_initPlugin")))
            {
                loaded = initializePlugin(initFunc);
                log.get(LogLevel::Debug) << "Initialized plugin '" <<
                    completePath << "'." << std::endl;
            }
            else
                log.get(LogLevel::Error) << "Failed to initialize plugin function for plugin '" <<
                    completePath << "'." << std::endl;
        }
    }

    return loaded;
}


void *PluginManager::createObject(const std::string& objectType)
{
    return s_instance.l_createObject(objectType);
}


void *PluginManager::l_createObject(const std::string& objectType)
{
    auto find([this, &objectType]()->bool
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_plugins.count(objectType);
    });


    void *obj(0);
    if (find() || (guessLoadByPath(objectType) && find()))
    {
        PF_CreateFunc f;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            f = m_plugins[objectType].createFunc;
        }
        obj = f();
    }

    return obj;
}


DynamicLibrary *PluginManager::loadLibrary(const std::string& path,
    std::string& errorString)
{
    DynamicLibrary *d = DynamicLibrary::load(path, errorString);

    if (d)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_dynamicLibraryMap[FileUtils::toAbsolutePath(path)] = DynLibPtr(d);
    }

    return d;
}

bool PluginManager::libraryLoaded(const std::string& path)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string p = FileUtils::toAbsolutePath(path);
    return Utils::contains(m_dynamicLibraryMap, p);
}

} // namespace pdal

