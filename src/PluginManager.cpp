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

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <pdal/pdal_defines.h>
#include <pdal/util/FileUtils.hpp>
#include <pdal/Utils.hpp>

#include "DynamicLibrary.h"

#include <iostream> //temp
#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

#if defined(PDAL_PLATFORM_OSX)
    static std::string dynamicLibraryExtension(".dylib");
#elif defined(PDAL_PLATFORM_LINUX)
    static std::string dynamicLibraryExtension(".so");
#elif defined(PDAL_PLATFORM_WIN32)
    static std::string dynamicLibraryExtension(".dll");
#endif

static bool isValid(const PF_RegisterParams * params)
{
    if (!params || !params->createFunc || !params->destroyFunc)
        return false;

    return true;
}

int32_t PluginManager::registerObject(const std::string & objectType, const PF_RegisterParams * params)
{
    if (!isValid(params))
        return -1;

    PluginManager & pm = PluginManager::getInstance();

    // skipped version check for now

    if (pm.exactMatchMap_.find(objectType) != pm.exactMatchMap_.end())
        return -1;

    pm.exactMatchMap_[objectType] = *params;
    return 0;
}

PluginManager & PluginManager::getInstance()
{
    static PluginManager instance;
    return instance;
}

int32_t PluginManager::loadAll(PF_PluginType type)
{
    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // If we don't have a driver path, we'll default to /usr/local/lib and lib

    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH << ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir,
        boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        loadAll(pluginPath, type);
    }

    return 0;
}

int32_t PluginManager::loadAll(const std::string & pluginDirectory, PF_PluginType type)
{
    if (pluginDirectory.empty())
        return -1;

    if (!boost::filesystem::exists(pluginDirectory) || !boost::filesystem::is_directory(pluginDirectory))
        return -1;

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

    return 0;
}

int32_t PluginManager::initializePlugin(PF_InitFunc initFunc)
{
    PluginManager & pm = PluginManager::getInstance();

    PF_ExitFunc exitFunc = initFunc();
    if (!exitFunc)
        return -1;

    pm.exitFuncVec_.push_back(exitFunc);
    return 0;
}

PluginManager::PluginManager() : inInitializePlugin_(false)
{}

PluginManager::~PluginManager()
{
    shutdown();
}

int32_t PluginManager::shutdown()
{
    int32_t result = 0;
    for (auto const& func : exitFuncVec_)
    {
        try
        {
            result = (*func)();
        }
        catch (...)
        {
            result = 01;
        }
    }

    dynamicLibraryMap_.clear();
    exactMatchMap_.clear();
    exitFuncVec_.clear();

    return result;
}

int32_t PluginManager::guessLoadByPath(const std::string& driverName)
{
    // parse the driver name into an expected pluginName, e.g., writers.las => libpdal_plugin_writer_las
    std::vector<std::string> driverNameVec;
    boost::algorithm::split(driverNameVec, driverName,
        boost::algorithm::is_any_of("."), boost::algorithm::token_compress_on);

    std::string pluginName = "libpdal_plugin_" + driverNameVec[0] + "_" + driverNameVec[1];

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // If we don't have a driver path, we'll default to /usr/local/lib and lib

    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH << ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir,
        boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        boost::filesystem::path path(pluginPath);

        if (path.empty())
            continue;

        if (!boost::filesystem::exists(path) || !boost::filesystem::is_directory(path))
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

            PF_PluginType type;
            if (driverNameVec[0] == "readers")
                type = PF_PluginType_Reader;
            else if (driverNameVec[0] == "kernels")
                type = PF_PluginType_Kernel;
            else if (driverNameVec[0] == "filters")
                type = PF_PluginType_Filter;
            else if (driverNameVec[0] == "writers")
                type = PF_PluginType_Writer;

            loadByPath(full_path.string(), type);
        }
   }

    return 0;
}

int32_t PluginManager::loadByPath(const std::string& pluginPath, PF_PluginType type)
{
    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color

    boost::filesystem::path path(pluginPath);

    bool isValid = false;
    if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_kernel"))
    {
        if (type == PF_PluginType_Kernel) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_filter"))
    {
        if (type == PF_PluginType_Filter) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_reader"))
    {
        if (type == PF_PluginType_Reader) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_writer"))
    {
        if (type == PF_PluginType_Writer) { isValid = true; }
    }

    if (!isValid)
        return -1;

    if (dynamicLibraryMap_.find(path.string()) != dynamicLibraryMap_.end())
        return -1;

    std::string errorString;
    DynamicLibrary * d = loadLibrary(boost::filesystem::complete(path).string(), errorString);
    if (!d)
        return -1;

    PF_InitFunc initFunc = (PF_InitFunc)(d->getSymbol("PF_initPlugin"));
    if (!initFunc)
        return -1;

    int32_t res = initializePlugin(initFunc);
    if (res < 0)
        return res;

    return 0;
}

void * PluginManager::createObject(const std::string & objectType)
{
    if (exactMatchMap_.find(objectType) != exactMatchMap_.end())
    {
        PF_RegisterParams & rp = exactMatchMap_[objectType];
        void * object = rp.createFunc();
        if (object)
        {
            return object;
        }
    }

    return NULL;
}

DynamicLibrary * PluginManager::loadLibrary(const std::string & path, std::string & errorString)
{
    DynamicLibrary * d = DynamicLibrary::load(path, errorString);
    if (!d)
        return NULL;

    dynamicLibraryMap_[boost::filesystem::complete(path).string()] = std::shared_ptr<DynamicLibrary>(d);
    return d;
}

const PluginManager::RegistrationMap & PluginManager::getRegistrationMap()
{
    return exactMatchMap_;
}

} // namespace pdal

