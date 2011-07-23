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

#include <pdal/StageFactory.hpp>

#include <pdal/Filter.hpp>
#include <pdal/MultiFilter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/drivers/oci/Reader.hpp>
#include <pdal/drivers/qfit/Reader.hpp>
#include <pdal/drivers/terrasolid/Reader.hpp>

#include <pdal/drivers/faux/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/liblas/Writer.hpp>
#include <pdal/drivers/oci/Writer.hpp>

#include <pdal/filters/ByteSwapFilter.hpp>
#include <pdal/filters/CacheFilter.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/ColorFilter.hpp>
#include <pdal/filters/CropFilter.hpp>
#include <pdal/filters/DecimationFilter.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>
#include <pdal/filters/ScalingFilter.hpp>

#include <pdal/filters/MosaicFilter.hpp>

#include <boost/shared_ptr.hpp>


namespace pdal
{

#define MAKE_READER_CREATOR(T, FullT) \
    Reader* create_##T(const Options& options) { return new FullT(options); }

#define MAKE_FILTER_CREATOR(T, FullT) \
    Filter* create_##T(const Stage& prevStage, const Options& options) { return new FullT(prevStage, options); }

#define MAKE_MULTIFILTER_CREATOR(T, FullT) \
    MultiFilter* create_##T(const std::vector<const Stage*>& prevStages, const Options& options) { return new FullT(prevStages, options); }

#define MAKE_WRITER_CREATOR(T, FullT) \
    Writer* create_##T(const Stage& prevStage, const Options& options) { return new FullT(prevStage, options); }

MAKE_READER_CREATOR(FauxReader, pdal::drivers::faux::Reader)
MAKE_READER_CREATOR(LasReader, pdal::drivers::las::LasReader);
MAKE_READER_CREATOR(LiblasReader, pdal::drivers::liblas::LiblasReader)
MAKE_READER_CREATOR(OciReader, pdal::drivers::oci::Reader)
MAKE_READER_CREATOR(QfitReader, pdal::drivers::qfit::Reader)
MAKE_READER_CREATOR(TerrasolidReader, pdal::drivers::terrasolid::Reader)

MAKE_FILTER_CREATOR(ByteSwapFilter, pdal::filters::ByteSwapFilter)
MAKE_FILTER_CREATOR(CacheFilter, pdal::filters::CacheFilter)
MAKE_FILTER_CREATOR(Chipper, pdal::filters::Chipper)
MAKE_FILTER_CREATOR(ColorFilter, pdal::filters::ColorFilter)
MAKE_FILTER_CREATOR(CropFilter, pdal::filters::CropFilter)
MAKE_FILTER_CREATOR(DecimationFilter, pdal::filters::DecimationFilter)
MAKE_FILTER_CREATOR(ReprojectionFilter, pdal::filters::ReprojectionFilter)
MAKE_FILTER_CREATOR(ScalingFilter, pdal::filters::ScalingFilter)

MAKE_MULTIFILTER_CREATOR(MosaicFilter, pdal::filters::MosaicFilter)

MAKE_WRITER_CREATOR(FauxWriter, pdal::drivers::faux::Writer)
MAKE_WRITER_CREATOR(LasWriter, pdal::drivers::las::LasWriter)
MAKE_WRITER_CREATOR(LiblasWriter, pdal::drivers::liblas::LiblasWriter)
MAKE_WRITER_CREATOR(OciWriter, pdal::drivers::oci::Writer)



StageFactory::StageFactory()
{
    registerKnownReaders();
    registerKnownFilters();
    registerKnownMultiFilters();
    registerKnownWriters();

    return;
}


Reader* StageFactory::createReader(const std::string& type, const Options& options)
{
    ReaderCreator* f = getReaderCreator(type);
    Reader* stage = f(options);
    return stage;
}


Filter* StageFactory::createFilter(const std::string& type, const Stage& prevStage, const Options& options)
{
    FilterCreator* f = getFilterCreator(type);
    Filter* stage = f(prevStage, options);
    return stage;
}


MultiFilter* StageFactory::createMultiFilter(const std::string& type, const std::vector<const Stage*>& prevStages, const Options& options)
{
    MultiFilterCreator* f = getMultiFilterCreator(type);
    MultiFilter* stage = f(prevStages, options);
    return stage;
}


Writer* StageFactory::createWriter(const std::string& type, const Stage& prevStage, const Options& options)
{
    WriterCreator* f = getWriterCreator(type);
    Writer* stage = f(prevStage, options);
    return stage;
}


template<typename T>
static T* findFirst(const std::string& type, std::map<std::string, T*> list)
{
    typename std::map<std::string, T*>::const_iterator iter = list.find(type);
    if (iter == list.end())
        return NULL;
    return (*iter).second;
}


StageFactory::ReaderCreator* StageFactory::getReaderCreator(const std::string& type) const
{
    return findFirst<ReaderCreator>(type, m_readerCreators);
}


StageFactory::FilterCreator* StageFactory::getFilterCreator(const std::string& type) const
{
    return findFirst<FilterCreator>(type, m_filterCreators);
}


StageFactory::MultiFilterCreator* StageFactory::getMultiFilterCreator(const std::string& type) const
{
    return findFirst<MultiFilterCreator>(type, m_multifilterCreators);
}


StageFactory::WriterCreator* StageFactory::getWriterCreator(const std::string& type) const
{
    return findFirst<WriterCreator>(type, m_writerCreators);
}


void StageFactory::registerReader(const std::string& type, ReaderCreator* f)
{
    std::pair<std::string, ReaderCreator*> p(type, f);
    m_readerCreators.insert(p);
}


void StageFactory::registerFilter(const std::string& type, FilterCreator* f)
{
    std::pair<std::string, FilterCreator*> p(type, f);
    m_filterCreators.insert(p);
}


void StageFactory::registerMultiFilter(const std::string& type, MultiFilterCreator* f)
{
    std::pair<std::string, MultiFilterCreator*> p(type, f);
    m_multifilterCreators.insert(p);
}


void StageFactory::registerWriter(const std::string& type, WriterCreator* f)
{
    std::pair<std::string, WriterCreator*> p(type, f);
    m_writerCreators.insert(p);
}


void StageFactory::registerKnownReaders()
{
#define REGISTER_READER(T, FullT) \
    registerReader(FullT::s_getName(), create_##T)

    REGISTER_READER(FauxReader, pdal::drivers::faux::Reader);
    REGISTER_READER(LasReader, pdal::drivers::las::LasReader);
    REGISTER_READER(LiblasReader, pdal::drivers::liblas::LiblasReader);
    REGISTER_READER(OciReader, pdal::drivers::oci::Reader);
    REGISTER_READER(QfitReader, pdal::drivers::qfit::Reader);
    REGISTER_READER(TerrasolidReader, pdal::drivers::terrasolid::Reader);
}


void StageFactory::registerKnownFilters()
{
#define REGISTER_FILTER(T, FullT) \
    registerFilter(FullT::s_getName(), create_##T)

    REGISTER_FILTER(ByteSwapFilter, pdal::filters::ByteSwapFilter);
    REGISTER_FILTER(CacheFilter, pdal::filters::CacheFilter);
    REGISTER_FILTER(Chipper, pdal::filters::Chipper);
    REGISTER_FILTER(ColorFilter, pdal::filters::ColorFilter);
    REGISTER_FILTER(CropFilter, pdal::filters::CropFilter);
    REGISTER_FILTER(DecimationFilter, pdal::filters::DecimationFilter);
    REGISTER_FILTER(ReprojectionFilter, pdal::filters::ReprojectionFilter);
    REGISTER_FILTER(ScalingFilter, pdal::filters::ScalingFilter);
}


void StageFactory::registerKnownMultiFilters()
{
#define REGISTER_MULTIFILTER(T, FullT) \
    registerMultiFilter(FullT::s_getName(), create_##T)
    
    REGISTER_MULTIFILTER(MosaicFilter, pdal::filters::MosaicFilter);
}


void StageFactory::registerKnownWriters()
{
#define REGISTER_WRITER(T, FullT) \
    registerWriter(FullT::s_getName(), create_##T)

    REGISTER_WRITER(FauxWriter, pdal::drivers::faux::Writer);
    REGISTER_WRITER(LasWriter, pdal::drivers::las::LasWriter);
    REGISTER_WRITER(LiblasWriter, pdal::drivers::liblas::LiblasWriter);
    REGISTER_WRITER(OciWriter, pdal::drivers::oci::Writer);
}


} // namespace pdal
