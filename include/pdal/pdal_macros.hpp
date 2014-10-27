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

#include <pdal/pdal_internal.hpp>
//
// macros for creating the various stage types
//
#define MAKE_READER_CREATOR(T, FullT) \
    pdal::Reader* create_##T() \
        { pdal::Reader *r = new FullT(); return r; }
#define MAKE_FILTER_CREATOR(T, FullT) \
    pdal::Filter* create_##T() \
        { pdal::Filter *f = new FullT(); return f; }
#define MAKE_WRITER_CREATOR(T, FullT) \
    pdal::Writer* create_##T() \
        { pdal::Writer *w = new FullT(); return w; }
#define MAKE_KERNEL_CREATOR(T, FullT) \
    pdal::Kernel* create_##T() \
        { pdal::Kernel *k = new FullT(); return k; }

//
// macros to register the stage creators
//
#define REGISTER_WRITER(T, FullT) \
    registerDriverInfo<FullT>(); \
    registerWriter(FullT::s_getName(), create_##T)
#define REGISTER_READER(T, FullT) \
    registerDriverInfo<FullT>(); \
    registerReader(FullT::s_getName(), create_##T)
#define REGISTER_FILTER(T, FullT) \
    registerDriverInfo<FullT>(); \
    registerFilter(FullT::s_getName(), create_##T)
#define REGISTER_KERNEL(T, FullT) \
    registerKernelDriverInfo<FullT>(); \
    registerKernel(FullT::s_getName(), create_##T)

//
// macros to create the various plugins types
//
#define CREATE_READER_PLUGIN(T, FullT) \
    MAKE_READER_CREATOR(T, FullT) \
    PDAL_C_START PDAL_DLL void PDALRegister_reader_##T(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< FullT>(); \
        f.registerReader(FullT::s_getName(), create_##T); \
    } \
    PDAL_C_END \
    SET_READER_PLUGIN_VERSION(T)
#define CREATE_FILTER_PLUGIN(T, FullT) \
    MAKE_FILTER_CREATOR(T, FullT) \
    PDAL_C_START PDAL_DLL void PDALRegister_filter_##T(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< FullT>(); \
        f.registerFilter(FullT::s_getName(), create_##T); \
    } \
    PDAL_C_END \
    SET_FILTER_PLUGIN_VERSION(T)
#define CREATE_WRITER_PLUGIN(T, FullT) \
    MAKE_WRITER_CREATOR(T, FullT) \
    PDAL_C_START PDAL_DLL void PDALRegister_writer_##T(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< FullT>(); \
        f.registerWriter(FullT::s_getName(), create_##T); \
    } \
    PDAL_C_END \
    SET_WRITER_PLUGIN_VERSION(T)
#define CREATE_KERNEL_PLUGIN(T, FullT) \
    MAKE_KERNEL_CREATOR(T, FullT) \
    PDAL_C_START PDAL_DLL void PDALRegister_kernel_##T(void* factory) \
    { \
        pdal::KernelFactory& f = *(pdal::KernelFactory*) factory; \
        f.registerKernelDriverInfo< FullT>(); \
        f.registerKernel(FullT::s_getName(), create_##T); \
    } \
    PDAL_C_END \
    SET_KERNEL_PLUGIN_VERSION(T)

//
// macros to register the plugin creators
//
//#define REGISTER_READER_PLUGIN(T) \
//    PDAL_C_START \
//    PDAL_DLL void PDALRegister_reader_##T(void* factory); \
//    PDAL_C_END
//#define REGISTER_FILTER_PLUGIN(T) \
//    PDAL_C_START \
//    PDAL_DLL void PDALRegister_filter_##T(void* factory); \
//    PDAL_C_END
//#define REGISTER_WRITER_PLUGIN(T) \
//    PDAL_C_START \
//    PDAL_DLL void PDALRegister_writer_##T(void* factory); \
//    PDAL_C_END
//#define REGISTER_KERNEL_PLUGIN(T) \
//    PDAL_C_START \
//    PDAL_DLL void PDALRegister_kernel_##T(void* factory); \
//    PDAL_C_END

//
// macro to register the version of PDAL the plugin was linked against
//
#define SET_READER_PLUGIN_VERSION(T) \
    PDAL_C_START PDAL_DLL int PDALRegister_version_reader_##T() \
        { return PDAL_PLUGIN_VERSION; } \
    PDAL_C_END
#define SET_FILTER_PLUGIN_VERSION(T) \
    PDAL_C_START PDAL_DLL int PDALRegister_version_filter_##T() \
        { return PDAL_PLUGIN_VERSION; } \
    PDAL_C_END
#define SET_WRITER_PLUGIN_VERSION(T) \
    PDAL_C_START PDAL_DLL int PDALRegister_version_writer_##T() \
        { return PDAL_PLUGIN_VERSION; } \
    PDAL_C_END
#define SET_KERNEL_PLUGIN_VERSION(T) \
    PDAL_C_START PDAL_DLL int PDALRegister_version_kernel_##T() \
        { return PDAL_PLUGIN_VERSION; } \
    PDAL_C_END

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

