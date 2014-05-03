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

#ifndef INCLUDED_PDAL_MACROS_HPP
#define INCLUDED_PDAL_MACROS_HPP

#include <pdal/pdal_internal.hpp>
//
// macros for creating the various stage types
//
#define MAKE_READER_CREATOR(T, FullT) \
    pdal::Reader* create_##T(const pdal::Options& options) \
        { \
            return new FullT(options); \
        } 
            
#define MAKE_FILTER_CREATOR(T, FullT) \
    pdal::Filter* create_##T(pdal::Stage& prevStage, const pdal::Options& options) \
        { pdal::Filter *f = new FullT(options); f->setInput(&prevStage); return f; }
#define MAKE_MULTIFILTER_CREATOR(T, FullT) \
    pdal::MultiFilter* create_##T(const std::vector<pdal::Stage*>& prevStages, const pdal::Options& options) \
        { pdal::MultiFilter *mf = new FullT(options); mf->setInput(prevStages); return mf; }
#define MAKE_WRITER_CREATOR(T, FullT) \
    pdal::Writer* create_##T(pdal::Stage& prevStage, const pdal::Options& options) \
        { pdal::Writer *w = new FullT(options); w->setInput(&prevStage); return w; }

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
#define REGISTER_MULTIFILTER(T, FullT) \
    registerDriverInfo<FullT>(); \
    registerMultiFilter(FullT::s_getName(), create_##T)

#define CREATE_READER_PLUGIN(DriverName, DriverFullType) \
    PDAL_C_START PDAL_DLL void PDALRegister_reader_##DriverName(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< DriverFullType>(); \
        f.registerReader(DriverFullType::s_getName(), create_##DriverName##Reader); \
    } \
    PDAL_C_END 

#define CREATE_FILTER_PLUGIN(DriverName, DriverFullType) \
    PDAL_C_START PDAL_DLL void PDALRegister_filter_##DriverName(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< DriverFullType>(); \
        f.registerFilter(DriverFullType::s_getName(), create_##DriverName##Filter); \
    } \
    PDAL_C_END 

#define CREATE_MULTIFILTER_PLUGIN(DriverName, DriverFullType) \
    PDAL_C_START PDAL_DLL void PDALRegister_multifilter_##DriverName(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< DriverFullType>(); \
        f.registerMultiFilter(DriverFullType::s_getName(), create_##DriverName##MultiFilter); \
    } \
    PDAL_C_END 

#define CREATE_WRITER_PLUGIN(DriverName, DriverFullType) \
    PDAL_C_START PDAL_DLL void PDALRegister_writer_##DriverName(void* factory) \
    { \
        pdal::StageFactory& f = *(pdal::StageFactory*) factory; \
        f.registerDriverInfo< DriverFullType>(); \
        f.registerWriter(DriverFullType::s_getName(), create_##DriverName##Writer); \
    } \
    PDAL_C_END 

#define SET_PLUGIN_VERSION(DriverName) \
    PDAL_C_START PDAL_DLL int PDALRegister_version_##DriverName() \
    { \
        return PDAL_PLUGIN_VERSION; \
    } \
    PDAL_C_END 


#endif

