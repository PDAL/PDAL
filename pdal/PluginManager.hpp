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
#include <pdal/pdal_internal.hpp>
#include <pdal/plugin.hpp>

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

class PDAL_DLL PluginManager
{
    FRIEND_TEST(PluginManagerTest, SearchPaths);

    typedef std::shared_ptr<DynamicLibrary> DynLibPtr;
    typedef std::map<std::string, DynLibPtr> DynamicLibraryMap;
    typedef std::vector<PF_ExitFunc> ExitFuncVec;
    typedef std::map<std::string, PF_RegisterParams> RegistrationInfoMap;

public:
    PluginManager();
    ~PluginManager();

    static std::string description(const std::string& name);
    static std::string link(const std::string& name);
    static bool registerObject(const std::string& name,
        const PF_RegisterParams *params);
    static bool initializePlugin(PF_InitFunc initFunc);
    static bool loadPlugin(const std::string& pluginFilename);
    static void loadAll(int type);
    static void *createObject(const std::string& objectType);
    static StringList names(int typeMask);
    static void setLog(LogPtr& log);

private:
    // These functions return true if successful.
    bool shutdown();
    bool guessLoadByPath(const std::string & driverName);
    bool loadByPath(const std::string & path, PF_PluginType type);
    bool libraryLoaded(const std::string& path);
    void loadAll(const std::string& pluginDirectory, int type);
    bool l_initializePlugin(PF_InitFunc initFunc);
    void *l_createObject(const std::string& objectType);
    bool l_registerObject(const std::string& name,
        const PF_RegisterParams *params);
    bool l_loadPlugin(const std::string& pluginFilename);
    void l_loadAll(int type);
    StringList l_names(int typeMask);
    std::string l_description(const std::string& name);
    std::string l_link(const std::string& name);
    DynamicLibrary *loadLibrary(const std::string& path,
        std::string& errorString);

    static StringList test_pluginSearchPaths();

    PF_PluginAPI_Version m_version;
    DynamicLibraryMap m_dynamicLibraryMap;
    ExitFuncVec m_exitFuncVec;
    RegistrationInfoMap m_plugins;
    std::mutex m_mutex;
    LogPtr m_log;

    // Disable copy/assignment.
    PluginManager(const PluginManager&);
    PluginManager& operator=(const PluginManager&);
};

} // namespace pdal

