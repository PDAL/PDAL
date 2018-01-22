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

#include "private/DynamicLibrary.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

namespace
{

/**
static PluginManager<Stage> s_stageInstance({"reader", "writer", "filter"});
static PluginManager<Kernel> s_kernelInstance({"kernel"});
**/

#if defined(__APPLE__) && defined(__MACH__)
    const std::string dynamicLibraryExtension(".dylib");
#elif defined(__linux__) || defined(__FreeBSD__) || defined(__DragonFly__) || defined(__FreeBSD_kernel__) || defined(__GNU__)
    const std::string dynamicLibraryExtension(".so");
#elif defined _WIN32
    const std::string dynamicLibraryExtension(".dll");
#endif

StringList pluginSearchPaths()
{
    StringList searchPaths;
    std::string envOverride;

    Utils::getenv("PDAL_DRIVER_PATH", envOverride);

    if (!envOverride.empty())
        searchPaths = Utils::split2(envOverride, ':');
    else
    {
        StringList standardPaths { ".", "./lib", "../lib", "./bin", "../bin" };
        for (std::string& s : standardPaths)
        {
            if (FileUtils::toAbsolutePath(s) !=
                FileUtils::toAbsolutePath(PDAL_PLUGIN_INSTALL_PATH))
                searchPaths.push_back(s);
        }
        searchPaths.push_back(PDAL_PLUGIN_INSTALL_PATH);
    }
    return searchPaths;
}
} // unnamed namespace;

PluginManager<Stage> s_stageInstance({"reader", "writer", "filter"});
PluginManager<Kernel> s_kernelInstance({"kernel"});

template <typename T>
bool PluginManager<T>::pluginNameValid(const std::string& pathname)
{
    std::string base("libpdal_plugin_");
    for (std::string& suffix : m_suffixes)
        if (Utils::startsWith(pathname, base + suffix))
            return true;
    return false;
}


template <typename T>
void PluginManager<T>::loadAll()
{
    m_instance->l_loadAll();
}


template <typename T>
void PluginManager<T>::l_loadAll()
{
    for (const auto& pluginPath : pluginSearchPaths())
        loadAll(pluginPath);
}


template <typename T>
StringList PluginManager<T>::test_pluginSearchPaths()
{
    return pluginSearchPaths();
}


template <typename T>
StringList PluginManager<T>::names()
{
    return m_instance->l_names();
}


template <typename T>
void PluginManager<T>::setLog(LogPtr& log)
{
    m_instance->m_log = log;
}


template <typename T>
StringList PluginManager<T>::l_names()
{
    StringList l;

    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto p : m_plugins)
        l.push_back(p.first);
    return l;
}


/**
  Return a string with an HTML link to plugin documentation.

  \param name  Plugin name.
  \return  Link to documentation.
*/
template <typename T>
std::string PluginManager<T>::link(const std::string& name)
{
    return m_instance->l_link(name);
}


template <typename T>
std::string PluginManager<T>::l_link(const std::string& name)
{
    std::string link;

    std::lock_guard<std::mutex> lock(m_mutex);
    auto ei = m_plugins.find(name);
    if (ei != m_plugins.end())
        link = ei->second.link;
    return link;
}


/**
  Return a short description of the plugin.

  \param name  Plugin name.
  \return  Description string.
*/
template <typename T>
std::string PluginManager<T>::description(const std::string& name)
{
    return m_instance->l_description(name);
}


template <typename T>
std::string PluginManager<T>::l_description(const std::string& name)
{
    std::string descrip;

    std::lock_guard<std::mutex> lock(m_mutex);
    auto ei = m_plugins.find(name);
    if (ei != m_plugins.end())
        descrip = ei->second.description;
    return descrip;
}


/**
  Load all plugins in a directory.

  \param pluginDirectory  Name of plugin directory.
*/
template <typename T>
void PluginManager<T>::loadAll(const std::string& pluginDirectory)
{
    const bool pluginDirectoryValid = pluginDirectory.size() &&
        (FileUtils::fileExists(pluginDirectory) ||
            FileUtils::isDirectory(pluginDirectory));

    if (pluginDirectoryValid)
    {
        m_log->get(LogLevel::Debug) << "Loading plugins from directory " <<
            pluginDirectory << std::endl;
        StringList files = FileUtils::directoryList(pluginDirectory);
        for (auto file : files)
        {
            if ((FileUtils::extension(file) == dynamicLibraryExtension) &&
                !FileUtils::isDirectory(file))
                loadByPath(file);
        }
    }
}


template <typename T>
bool PluginManager<T>::initializePlugin(PF_InitFunc initFunc)
{
    return m_instance->l_initializePlugin(initFunc);
}


template <typename T>
bool PluginManager<T>::l_initializePlugin(PF_InitFunc initFunc)
{
    initFunc();
    return true;
}


template <typename T>
PluginManager<T>::PluginManager(const StringList& suffixes) :
    m_suffixes(suffixes), m_log(new Log("PDAL", &std::clog))
{
    m_instance = this;
}


template <typename T>
PluginManager<T>::~PluginManager()
{
    if (!shutdown())
        m_log->get(LogLevel::Error) <<
            "Error destroying PluginManager" << std::endl;
}


template <typename T>
bool PluginManager<T>::shutdown()
{
    std::lock_guard<std::mutex> lock(m_mutex);

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

    return true;
}


/**
  Load a plugin by providing a path to the shared object.

  \param driverFileName  Path to driver to load.
*/
template <typename T>
bool PluginManager<T>::loadPlugin(const std::string& driverFileName)
{
    return m_instance->l_loadPlugin(driverFileName);
}


template <typename T>
bool PluginManager<T>::l_loadPlugin(const std::string& driverFileName)
{
    if ((FileUtils::extension(driverFileName) != dynamicLibraryExtension) ||
            FileUtils::isDirectory(driverFileName))
        return false;

    return loadByPath(driverFileName);
}


template <typename T>
bool PluginManager<T>::guessLoadByPath(const std::string& driverName)
{
    // parse the driver name into an expected pluginName, e.g.,
    // writers.las => libpdal_plugin_writer_las

    StringList driverNameVec = Utils::split2(driverName, '.');
    if (driverNameVec.size() != 2)
    {
        m_log->get(LogLevel::Error) <<
           "Invalid PDAL driver name '" << driverName << "'." << std::endl;
        return false;
    }
    // The substr() removes the 's' from the end of the type.
    std::string pluginName = "libpdal_plugin_" +
        driverNameVec[0].substr(0, driverNameVec[0].size() - 1) + "_" +
        driverNameVec[1] + dynamicLibraryExtension;

    for (const auto& pluginPath : pluginSearchPaths())
    {
#ifdef _WIN32
        char pathsep('\\');
#else
        char pathsep('/');
#endif
        std::string file = pluginPath + pathsep + pluginName;
        if (FileUtils::fileExists(file) &&
            !FileUtils::isDirectory(file))
            return loadByPath(file);
    }

    return false;
}


/**
  Load a specific plugin.

  \param pluginPath  Path to plugin to load.
  \return  \c true on success, \c false on error
*/
template <typename T>
bool PluginManager<T>::loadByPath(const std::string& pluginPath)
{
    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color

    bool loaded(false);

    std::string filename = Utils::tolower(FileUtils::getFilename(pluginPath));

    // If we are a valid type, and we're not yet already
    // loaded in the LibraryMap, load it.

    if (pluginNameValid(filename) && !libraryLoaded(pluginPath))
    {
        std::string errorString;
        auto completePath(FileUtils::toAbsolutePath(pluginPath));

        m_log->get(LogLevel::Debug) << "Attempting to load plugin '" <<
            completePath << "'." << std::endl;

        if (DynamicLibrary *d = loadLibrary(completePath, errorString))
        {
            m_log->get(LogLevel::Debug) << "Loaded plugin '" << completePath <<
                "'." << std::endl;
            if (PF_InitFunc initFunc =
                    (PF_InitFunc)(d->getSymbol("PF_initPlugin")))
            {
                loaded = initializePlugin(initFunc);
                m_log->get(LogLevel::Debug) << "Initialized plugin '" <<
                    completePath << "'." << std::endl;
            }
            else
                m_log->get(LogLevel::Error) <<
                    "Failed to initialize plugin function for plugin '" <<
                    completePath << "'." << std::endl;
        }
        else
            m_log->get(LogLevel::Error) << "Plugin '" << completePath <<
                "' found but failed to load: " << errorString << std::endl;
    }

    return loaded;
}


template <typename T>
T *PluginManager<T>::createObject(const std::string& objectType)
{
    return m_instance->l_createObject(objectType);
}


template <typename T>
T *PluginManager<T>::l_createObject(const std::string& objectType)
{
    auto find([this, &objectType]()->bool
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_plugins.count(objectType);
    });

    if (find() || (guessLoadByPath(objectType) && find()))
    {
        // Kernels contain a PipelineManager, which may load other plugins,
        // so the lock must be released prior to invoking the ctor to
        // avoid deadlock.
        std::function<T *()> f;
        {
            // Lock must be released before the creation function is invoked.
            std::lock_guard<std::mutex> lock(m_mutex);
            f = m_plugins[objectType].create;
        }
        return f();
    }
    return nullptr;
}

template <typename T>
DynamicLibrary *PluginManager<T>::loadLibrary(const std::string& path,
    std::string& errorString)
{
    DynamicLibrary *d = DynamicLibrary::load(path, errorString);

    if (d)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_dynamicLibraryMap[FileUtils::toAbsolutePath(path)] = DynLibPtr(d);
    }
    else
    {
        m_log->get(LogLevel::Error) << "Can't load library " << path <<
            ": " << errorString;
    }

    return d;
}


template <typename T>
bool PluginManager<T>::libraryLoaded(const std::string& path)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string p = FileUtils::toAbsolutePath(path);
    return Utils::contains(m_dynamicLibraryMap, p);
}

class Stage;
class Kernel;

template class PluginManager<Stage>;
template class PluginManager<Kernel>;


} // namespace pdal

