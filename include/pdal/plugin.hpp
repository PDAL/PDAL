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

// plugin.h was modeled very closely after the work of Gigi Sayfan in the Dr.
// Dobbs article:
// http://www.drdobbs.com/cpp/building-your-own-plugin-framework-part/206503957
// The original work was released under the Apache License v2.

#pragma once

#include <string>

#include <pdal/pdal_export.hpp>

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum PF_PluginType
{
    PF_PluginType_Kernel,
    PF_PluginType_Reader,
    PF_PluginType_Filter,
    PF_PluginType_Writer
} PF_PluginType;

typedef struct PF_PluginAPI_Version
{
    int32_t major;
    int32_t minor;
} PF_PluginAPI_Version;

typedef void * (*PF_CreateFunc)();
typedef int32_t (*PF_DestroyFunc)(void *);

typedef struct PF_RegisterParams
{
    PF_PluginAPI_Version version;
    PF_CreateFunc createFunc;
    PF_DestroyFunc destroyFunc;
    std::string description;
    std::string link;
    PF_PluginType pluginType;
} PF_RegisterParams;

typedef int32_t (*PF_ExitFunc)();
typedef PF_ExitFunc (*PF_InitFunc)();

#ifndef PDAL_DLL
  #ifdef _WIN32
    #define PDAL_DLL __declspec(dllimport)
  #else
    #define PDAL_DLL
  #endif
#endif

extern
#ifdef __cplusplus
"C"
#endif
PDAL_DLL PF_ExitFunc PF_initPlugin();

#ifdef __cplusplus
}
#endif

