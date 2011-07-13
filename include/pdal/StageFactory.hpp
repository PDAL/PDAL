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

#include <boost/shared_ptr.hpp>

#include <vector>
#include <map>
#include <string>


namespace pdal
{

class Stage;
class Reader;
class Filter;
class MultiFilter;
class Writer;
class Options;


class PDAL_DLL StageFactory
{
public:
    typedef Reader* readerCreatorFunction(const Options&);
    typedef Filter* filterCreatorFunction(boost::uint32_t prevStage, const Options&);
    typedef MultiFilter* multifilterCreatorFunction(const std::vector<boost::uint32_t>& prevStage, const Options&);
    typedef Writer* writerCreatorFunction(boost::uint32_t prevStage, const Options&);

public:
    StageFactory();

    boost::shared_ptr<Reader> createReader(const std::string& type, const Options& options);
    boost::shared_ptr<Filter> createFilter(const std::string& type, boost::uint32_t prevStage, const Options& options);
    boost::shared_ptr<MultiFilter> createMultiFilter(const std::string& type, const std::vector<boost::uint32_t>& prevStage, const Options& options);
    boost::shared_ptr<Writer> createWriter(const std::string& type, boost::uint32_t prevStage, const Options& options);

    void registerReader(const std::string& type, readerCreatorFunction* f);
    void registerFilter(const std::string& type, filterCreatorFunction* f);
    void registerMultiFilter(const std::string& type, multifilterCreatorFunction* f);
    void registerWriter(const std::string& type, writerCreatorFunction* f);

private:
    readerCreatorFunction* getReaderCreator(const std::string& type);
    filterCreatorFunction* getFilterCreator(const std::string& type);
    multifilterCreatorFunction* getMultiFilterCreator(const std::string& type);
    writerCreatorFunction* getWriterCreator(const std::string& type);

    void registerKnownStages();

    std::map<std::string, readerCreatorFunction*> m_readerCreators;
    std::map<std::string, filterCreatorFunction*> m_filterCreators;
    std::map<std::string, multifilterCreatorFunction*> m_multifilterCreators;
    std::map<std::string, writerCreatorFunction*> m_writerCreators;

    StageFactory& operator=(const StageFactory&); // not implemented
    StageFactory(const StageFactory&); // not implemented
};


} // namespace pdal

#endif
