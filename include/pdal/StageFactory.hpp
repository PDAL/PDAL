#if 0
/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#ifndef INCLUDED_STAGEFACTORY_HPP
#define INCLUDED_STAGEFACTORY_HPP

#include <pdal/pdal.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/MultiFilter.hpp>
#include <pdal/Writer.hpp>

#include <boost/shared_ptr.hpp>

#include <vector>
#include <map>
#include <string>


namespace pdal
{

class Options;


// This class provides a mechanism for creating Stage objects using only a
// string (the stage type name) and an Options block.
//
// We keep a list of (stage type name, creation function) pairs, which
// acts as a registry of creator functions.  The list is initialized with 
// the core stages we know about (I wish C++ had anonymous functions.).
// We allow the user to add his own "external" drivers to the registry list
// as well.
//
// We use 4 different functions for each kind of operation, since we have
// 4 types of derived classes from Stage and they all have slightly different
// parameters.  That makes it kinda messy.

class PDAL_DLL StageFactory
{
public:
    typedef Reader* ReaderCreator(const Options&);
    typedef Filter* FilterCreator(const DataStagePtr& prevStage, const Options&);
    typedef MultiFilter* MultiFilterCreator(const std::vector<const DataStagePtr>& prevStages, const Options&);
    typedef Writer* WriterCreator(const DataStagePtr& prevStage, const Options&);
    
    typedef std::map<std::string, ReaderCreator*> ReaderCreatorList;
    typedef std::map<std::string, FilterCreator*> FilterCreatorList;
    typedef std::map<std::string, MultiFilterCreator*> MultiFilterCreatorList;
    typedef std::map<std::string, WriterCreator*> WriterCreatorList;

public:
    StageFactory();

    ReaderPtr createReader(const std::string& type, const Options& options);
    FilterPtr createFilter(const std::string& type, const DataStagePtr& prevStage, const Options& options);
    MultiFilterPtr createMultiFilter(const std::string& type, const std::vector<const DataStagePtr>& prevStages, const Options& options);
    WriterPtr createWriter(const std::string& type, const DataStagePtr& prevStage, const Options& options);

    void registerReader(const std::string& type, ReaderCreator* f);
    void registerFilter(const std::string& type, FilterCreator* f);
    void registerMultiFilter(const std::string& type, MultiFilterCreator* f);
    void registerWriter(const std::string& type, WriterCreator* f);

private:
    ReaderCreator* getReaderCreator(const std::string& type) const;
    FilterCreator* getFilterCreator(const std::string& type) const;
    MultiFilterCreator* getMultiFilterCreator(const std::string& type) const;
    WriterCreator* getWriterCreator(const std::string& type) const;

    void registerKnownReaders();
    void registerKnownFilters();
    void registerKnownMultiFilters();
    void registerKnownWriters();

    // these are the "registries" of the factory creator functions
    ReaderCreatorList m_readerCreators;
    FilterCreatorList m_filterCreators;
    MultiFilterCreatorList m_multifilterCreators;
    WriterCreatorList m_writerCreators;

    StageFactory& operator=(const StageFactory&); // not implemented
    StageFactory(const StageFactory&); // not implemented
};


} // namespace pdal

#endif
#endif
