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

#pragma once

#include <pdal/Log.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Kernel.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace pdal
{

class DynamicLibrary;

/*
 * I think PluginManager can eventually be a private header, only accessible
 * through the factories, but we'll leave it as public for now.
 */

template <typename T>
class PDAL_DLL PluginManager
{
    struct Info
    {
        std::string name;
        std::string link;
        std::string description;
        std::function<T *()> create;
    };
    FRIEND_TEST(PluginManagerTest, SearchPaths);

    typedef std::shared_ptr<DynamicLibrary> DynLibPtr;
    typedef std::map<std::string, DynLibPtr> DynamicLibraryMap;
    typedef std::map<std::string, Info> RegistrationInfoMap;

public:
    PluginManager(const PluginManager&) = delete;
    PluginManager& operator=(const PluginManager&) = delete;
    PluginManager(const StringList& suffixes);
    ~PluginManager();

    static std::string description(const std::string& name);
    static std::string link(const std::string& name);
    template <typename C>
    static void registerPlugin(const PluginInfo& info)
        { return m_instance->l_registerPlugin<C>(info); }
    static bool initializePlugin(PF_InitFunc initFunc);
    static bool loadPlugin(const std::string& pluginFilename);
    static void loadAll();
    static T *createObject(const std::string& objectType);
    static StringList names();
    static void setLog(LogPtr& log);

private:
    bool pluginNameValid(const std::string& pathname);
    bool shutdown();
    bool guessLoadByPath(const std::string & driverName);
    bool loadByPath(const std::string & path);
    bool libraryLoaded(const std::string& path);
    void loadAll(const std::string& pluginDirectory);
    bool l_initializePlugin(PF_InitFunc initFunc);
    T *l_createObject(const std::string& objectType);
    template <class C>
    void l_registerPlugin(const PluginInfo& pi)
    {
        auto f = []()
        {
            T *t = dynamic_cast<T *>(new C);
            return t;
        };
        Info info {pi.name, pi.link, pi.description, f};
        std::lock_guard<std::mutex> lock(m_mutex);
        m_plugins.insert(std::make_pair(pi.name, info));
    }
    bool l_loadPlugin(const std::string& pluginFilename);
    void l_loadAll();
    StringList l_names();
    std::string l_description(const std::string& name);
    std::string l_link(const std::string& name);
    DynamicLibrary *loadLibrary(const std::string& path,
        std::string& errorString);

    static StringList test_pluginSearchPaths();

    static PluginManager<T> *m_instance;
    StringList m_suffixes;
    DynamicLibraryMap m_dynamicLibraryMap;
    RegistrationInfoMap m_plugins;
    std::mutex m_mutex;
    LogPtr m_log;
};

template<typename T>
PluginManager<T> *PluginManager<T>::m_instance;

} // namespace pdal

