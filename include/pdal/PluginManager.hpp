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

#include <pdal/plugin.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace pdal
{

class DynamicLibrary;

/*
 * I think PluginManager can eventually be a private header, only accessible
 * through the factories, but we'll leave it as public for now.
 */
class PluginManager
{
    typedef std::map<std::string, std::shared_ptr<DynamicLibrary> > DynamicLibraryMap;
    typedef std::vector<PF_ExitFunc> ExitFuncVec;
    typedef std::vector<PF_RegisterParams> RegistrationVec;

public:
    typedef std::map<std::string, PF_RegisterParams> RegistrationMap;

    static PluginManager & getInstance();
    static int32_t initializePlugin(PF_InitFunc initFunc);
    int32_t loadAll(PF_PluginType type);
    int32_t loadAll(const std::string & pluginDirectory, PF_PluginType type);
    int32_t guessLoadByPath(const std::string & driverName);
    int32_t loadByPath(const std::string & path, PF_PluginType type);

    void * createObject(const std::string & objectType);

    int32_t shutdown();
    static int32_t registerObject(const std::string & objectType, const PF_RegisterParams * params);
    const RegistrationMap & getRegistrationMap();

private:
    ~PluginManager();
    PluginManager();
    PluginManager(const PluginManager &);

    DynamicLibrary * loadLibrary(const std::string & path, std::string & errorString);

    bool inInitializePlugin_;
    DynamicLibraryMap dynamicLibraryMap_;
    ExitFuncVec exitFuncVec_;

    RegistrationMap tempExactMatchMap_;

    RegistrationMap exactMatchMap_;
};

} // namespace pdal

