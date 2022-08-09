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

#include <pdal/PluginDirectory.hpp>
#include <pdal/PluginManager.hpp>

#include <pdal/pdal_config.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include "private/DynamicLibrary.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

// This is thread-safe in C++ 11.
template <typename T>
PluginManager<T>& PluginManager<T>::get()
{
    static PluginManager<T> instance;

    return instance;
}


template <typename T>
StageExtensions& PluginManager<T>::extensions()
{
    return get().m_extensions;
}


template <typename T>
StringList PluginManager<T>::names()
{
    return get().l_names();
}


template <typename T>
void PluginManager<T>::setLog(LogPtr& log)
{
    get().m_log = log;
}


template <typename T>
StringList PluginManager<T>::l_names()
{
    StringList l;

    std::lock_guard<std::mutex> lock(m_pluginMutex);
    for (auto p : m_plugins)
        l.push_back(p.first);
    return l;
}


/**
  Load all plugins.
*/
template <typename T>
void PluginManager<T>::loadAll()
{
    get().l_loadAll();
}


//Base template.
template <typename T>
void PluginManager<T>::l_loadAll()
{}


template<>
void PluginManager<Stage>::l_loadAll()
{
    PluginDirectory& dir = PluginDirectory::get();
    for (auto& di: dir.m_drivers)
        l_loadDynamic(di.first);
}


template<>
void PluginManager<Kernel>::l_loadAll()
{
    PluginDirectory& dir = PluginDirectory::get();
    for (auto& di: dir.m_kernels)
        l_loadDynamic(di.first);
}


/**
  Return a string with an HTML link to plugin documentation.

  \param name  Plugin name.
  \return  Link to documentation.
*/
template <typename T>
std::string PluginManager<T>::link(const std::string& name)
{
    // Compute URLs for stage documents from the name instead
    // of using the URLs that are in the PluginInfo struct
    //
    // For development builds, we always point to master
    // https://pdal.dev/en/master/stages/filters.reprojection.html
    //
    // For versioned (Release) builds, we point to the version number
    // https://pdal.dev/en/2.4.3/stages/filters.reprojection.html
    //
    std::string root("https://pdal.io/en/");

    std::string version("master");
    std::string sha = Config::sha1();
    if (Utils::iequals(sha, "Release"))
        version = Config::versionString();

    std::string base = root + version +"/stages/";
    return base + name + ".html";
}


template <typename T>
std::string PluginManager<T>::l_link(const std::string& name)
{
    std::string link;

    std::lock_guard<std::mutex> lock(m_pluginMutex);
    auto ei = m_plugins.find(name);

    // Try to find the plugin given the name and use
    // the computed link name when we do
    if (ei != m_plugins.end())
        return get().link(name);
    return std::string("");
}


/**
  Return a short description of the plugin.

  \param name  Plugin name.
  \return  Description string.
*/
template <typename T>
std::string PluginManager<T>::description(const std::string& name)
{
    return get().l_description(name);
}


template <typename T>
std::string PluginManager<T>::l_description(const std::string& name)
{
    std::string descrip;

    std::lock_guard<std::mutex> lock(m_pluginMutex);
    auto ei = m_plugins.find(name);
    if (ei != m_plugins.end())
        descrip = ei->second.description;
    return descrip;
}


template <typename T>
PluginManager<T>::PluginManager() : m_log(Log::makeLog("PDAL", &std::clog)),
    m_extensions(m_log)
{}


template <typename T>
PluginManager<T>::~PluginManager()
{
    shutdown();
}


template <typename T>
void PluginManager<T>::shutdown()
{
    std::lock_guard<std::mutex> lock(m_pluginMutex);

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
}


/**
  Load a plugin by providing a path to the shared object.

  \param driverFileName  Path to driver to load.
*/
template <typename T>
bool PluginManager<T>::loadPlugin(const std::string& driverFileName)
{
    return get().l_loadPlugin(driverFileName);
}


template <typename T>
bool PluginManager<T>::l_loadPlugin(const std::string& driverFileName)
{
    return loadByPath(driverFileName);
}

template<typename T>
std::string PluginManager<T>::getPath(const std::string& driver)
{
    return std::string();
}


template<>
std::string PluginManager<Stage>::getPath(const std::string& driver)
{
    PluginDirectory& dir = PluginDirectory::get();
    auto it = dir.m_drivers.find(driver);
    if (it == dir.m_drivers.end())
        return std::string();
    return it->second;
}


template<>
std::string PluginManager<Kernel>::getPath(const std::string& driver)
{
    PluginDirectory& dir = PluginDirectory::get();

    auto it = dir.m_kernels.find(driver);
    if (it == dir.m_kernels.end())
        return std::string();
    return it->second;
}

template <typename T>
bool PluginManager<T>::loadDynamic(const std::string& driverName)
{
    return get().l_loadDynamic(driverName);
}

template <typename T>
bool PluginManager<T>::l_loadDynamic(const std::string& driverName)
{
    // If the library pointer is already in the map, we've already loaded
    // the library.
    std::string path = getPath(driverName);
    if (path.empty())
    {
        m_log->get(LogLevel::Debug) << "No plugin file found for driver '" <<
            driverName << "'." << std::endl;
        return false;
    }
    return loadByPath(path);
}


/**
  Load a specific plugin.

  \param pluginPath  Path to plugin to load.
  \return  \c true on success, \c false on error
*/
template <typename T>
bool PluginManager<T>::loadByPath(const std::string& path)
{
    if (libraryLoaded(path))
        return true;

    m_log->get(LogLevel::Debug) << "Attempting to load plugin '" <<
        path << "'." << std::endl;
    DynamicLibrary *d = loadLibrary(path);
    if (!d)
        return false;

    m_log->get(LogLevel::Debug) << "Loaded plugin '" << path <<
        "'." << std::endl;
    PF_InitFunc initFunc;

    // This awfulness is to work around a warning that some compilers
    // generate when casting a void * to a function *.  See the example
    // in the dlsym manpage.
    *(void **)(&initFunc) = d->getSymbol("PF_initPlugin");
    if (!initFunc)
    {
        m_log->get(LogLevel::Debug) << "No symbol 'PF_initPlugin' found "
            "in plugin '" << path << "'." << std::endl;
        return false;
    }
    initFunc();
    m_log->get(LogLevel::Debug) << "Initialized plugin '" <<
        path << "'." << std::endl;

    return true;
}


template <typename T>
T *PluginManager<T>::createObject(const std::string& objectType)
{
    return get().l_createObject(objectType);
}


template <typename T>
T *PluginManager<T>::l_createObject(const std::string& objectType)
{
    // Static plugins already here.
    auto find([this, &objectType]()->bool
    {
        std::lock_guard<std::mutex> lock(m_pluginMutex);
        return m_plugins.count(objectType);
    });

    if (find() || (l_loadDynamic(objectType) && find()))
    {
        // Kernels contain a PipelineManager, which may load other plugins,
        // so the lock must be released prior to invoking the ctor to
        // avoid deadlock.
        std::function<T *()> f;
        {
            // Lock must be released before the creation function is invoked.
            std::lock_guard<std::mutex> lock(m_pluginMutex);
            f = m_plugins[objectType].create;
        }
        return f();
    }
    return nullptr;
}


template <typename T>
DynamicLibrary *PluginManager<T>::libraryLoaded(const std::string& path)
{
    std::lock_guard<std::mutex> lock(m_libMutex);
    auto it = m_dynamicLibraryMap.find(path);
    if (it == m_dynamicLibraryMap.end())
        return nullptr;
    return it->second.get();
}


template <typename T>
DynamicLibrary *PluginManager<T>::loadLibrary(const std::string& path)
{
    std::string errorString;
    DynamicLibrary *d;

    d = libraryLoaded(path);
    if (d)
        return d;

    d = DynamicLibrary::load(path, errorString);
    if (d)
    {
        std::lock_guard<std::mutex> lock(m_libMutex);
        m_dynamicLibraryMap[FileUtils::toAbsolutePath(path)] = DynLibPtr(d);
    }
    else
        m_log->get(LogLevel::Error) << "Can't load library " << path <<
            ": " << errorString;

    return d;
}

class Stage;
class Kernel;

template class PluginManager<Stage>;
template class PluginManager<Kernel>;


} // namespace pdal

