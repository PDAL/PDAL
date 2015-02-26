/******************************************************************************
* Copyright (c) 2012, Howard Butler (hobu.inc@gmail.com)
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

#pragma once

#include <pdal/pdal_export.hpp>
#include <pdal/plugin.h>
#include <pdal/PluginManager.hpp>

namespace {

typedef struct PluginInfo {
    std::string name;
    std::string description;
    std::string link;
} PluginInfo;

}

#define CREATE_SHARED_PLUGIN(T, type, info) \
    extern "C" PDAL_DLL int32_t ExitFunc() \
    { return 0; } \
    extern "C" PDAL_DLL PF_ExitFunc PF_initPlugin() \
    { \
        int res = 0; \
        PF_RegisterParams rp; \
        rp.createFunc = pdal::T::create; \
        rp.destroyFunc = pdal::T::destroy; \
        strcpy(rp.description, info.description.c_str()); \
        strcpy(rp.link, info.link.c_str()); \
        rp.pluginType = PF_PluginType_ ## type; \
        pdal::PluginManager & pm = pdal::PluginManager::getInstance(); \
        res = pm.registerObject(info.name.c_str(), &rp); \
        if (res < 0) \
            return NULL; \
        return ExitFunc; \
    } \
    void * pdal::T::create() { return new pdal::T(); } \
    int32_t pdal::T::destroy(void *p) \
    { \
        if (!p) \
            return -1; \
        delete (pdal::T *)p; \
        return 0; \
    }

#define CREATE_STATIC_PLUGIN(T, type, info) \
    extern "C" PDAL_DLL int32_t T ## _ExitFunc() \
    { return 0; } \
    extern "C" PDAL_DLL PF_ExitFunc T ## _InitPlugin() \
    { \
        int res = 0; \
        PF_RegisterParams rp; \
        rp.createFunc = pdal::T::create; \
        rp.destroyFunc = pdal::T::destroy; \
        strcpy(rp.description, info.description.c_str()); \
        strcpy(rp.link, info.link.c_str()); \
        rp.pluginType = PF_PluginType_ ## type; \
        pdal::PluginManager & pm = pdal::PluginManager::getInstance(); \
        res = pm.registerObject(info.name.c_str(), &rp); \
        if (res < 0) \
            return NULL; \
        return T ## _ExitFunc; \
    } \
    void * pdal::T::create() { return new pdal::T(); } \
    int32_t pdal::T::destroy(void *p) \
    { \
        if (!p) \
            return -1; \
        delete (pdal::T *)p; \
        return 0; \
    }

#ifdef _WIN32
inline long lround(double d)
{
    long l;

    if (d < 0)
        l = (long)ceil(d - .5);
    else
        l = (long)floor(d + .5);
    return l;
}
#endif

