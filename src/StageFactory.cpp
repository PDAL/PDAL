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
#ifdef PDAL_HAVE_LIBLAS
#include <pdal/drivers/liblas/Reader.hpp>
#endif
#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Reader.hpp>
#endif
#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/drivers/qfit/Reader.hpp>
#include <pdal/drivers/terrasolid/Reader.hpp>

#include <pdal/drivers/faux/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>
#ifdef PDAL_HAVE_LIBLAS
#include <pdal/drivers/liblas/Writer.hpp>
#endif
#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Writer.hpp>
#endif

#include <pdal/filters/Attribute.hpp>
#include <pdal/filters/ByteSwap.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/Color.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Decimation.hpp>
#include <pdal/filters/Reprojection.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/filters/StatsFilter.hpp>

#include <pdal/filters/MosaicFilter.hpp>

#include <boost/shared_ptr.hpp>

#include <sstream>

namespace pdal
{
    //
    // macros for creating the various stage types
    //
#define MAKE_READER_CREATOR(T, FullT) \
    Reader* create_##T(const Options& options) \
        { return new FullT(options); }
#define MAKE_FILTER_CREATOR(T, FullT) \
    Filter* create_##T(Stage& prevStage, const Options& options) \
        { return new FullT(prevStage, options); }
#define MAKE_MULTIFILTER_CREATOR(T, FullT) \
    MultiFilter* create_##T(const std::vector<Stage*>& prevStages, const Options& options) \
        { return new FullT(prevStages, options); }
#define MAKE_WRITER_CREATOR(T, FullT) \
    Writer* create_##T(Stage& prevStage, const Options& options) \
        { return new FullT(prevStage, options); }

    //
    // macros to register the stage creators
    //
#define REGISTER_WRITER(T, FullT) \
    registerWriter(FullT::s_getName(), create_##T)
#define REGISTER_READER(T, FullT) \
    registerReader(FullT::s_getName(), create_##T)
#define REGISTER_FILTER(T, FullT) \
    registerFilter(FullT::s_getName(), create_##T)
#define REGISTER_MULTIFILTER(T, FullT) \
    registerMultiFilter(FullT::s_getName(), create_##T)

    //
    // define the functions to create the readers
    //
    MAKE_READER_CREATOR(FauxReader, pdal::drivers::faux::Reader)
    MAKE_READER_CREATOR(LasReader, pdal::drivers::las::Reader)
#ifdef PDAL_HAVE_LIBLAS
    MAKE_READER_CREATOR(LiblasReader, pdal::drivers::liblas::Reader)
#endif
#ifdef PDAL_HAVE_ORACLE
    MAKE_READER_CREATOR(OciReader, pdal::drivers::oci::Reader)
#endif
    MAKE_READER_CREATOR(PipelineReader, pdal::drivers::pipeline::Reader)
    MAKE_READER_CREATOR(QfitReader, pdal::drivers::qfit::Reader)
    MAKE_READER_CREATOR(TerrasolidReader, pdal::drivers::terrasolid::Reader)

    //
    // define the functions to create the filters
    //
    MAKE_FILTER_CREATOR(Attribute, pdal::filters::Attribute)
    MAKE_FILTER_CREATOR(ByteSwap, pdal::filters::ByteSwap)
    MAKE_FILTER_CREATOR(Cache, pdal::filters::Cache)
    MAKE_FILTER_CREATOR(Chipper, pdal::filters::Chipper)
    MAKE_FILTER_CREATOR(Color, pdal::filters::Color)
    MAKE_FILTER_CREATOR(Crop, pdal::filters::Crop)
    MAKE_FILTER_CREATOR(Decimation, pdal::filters::Decimation)
    MAKE_FILTER_CREATOR(Descaling, pdal::filters::Descaling)
    MAKE_FILTER_CREATOR(InPlaceReprojection, pdal::filters::InPlaceReprojection)
    MAKE_FILTER_CREATOR(Reprojection, pdal::filters::Reprojection)
    MAKE_FILTER_CREATOR(Scaling, pdal::filters::Scaling)
    MAKE_FILTER_CREATOR(StatsFilter, pdal::filters::StatsFilter)

    //
    // define the functions to create the multifilters
    //
    MAKE_MULTIFILTER_CREATOR(MosaicFilter, pdal::filters::MosaicFilter)

    //
    // define the functions to create the writers
    //
    MAKE_WRITER_CREATOR(FauxWriter, pdal::drivers::faux::Writer)
    MAKE_WRITER_CREATOR(LasWriter, pdal::drivers::las::Writer)
#ifdef PDAL_HAVE_LIBLAS
    MAKE_WRITER_CREATOR(LiblasWriter, pdal::drivers::liblas::Writer)
#endif
#ifdef PDAL_HAVE_ORACLE
    MAKE_WRITER_CREATOR(OciWriter, pdal::drivers::oci::Writer)
#endif


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
    if (!f) {
        std::ostringstream oss;
        oss << "Unable to create reader for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }
    Reader* stage = f(options);
    return stage;
}


Filter* StageFactory::createFilter(const std::string& type, Stage& prevStage, const Options& options)
{
    FilterCreator* f = getFilterCreator(type);
    if (!f) {
        std::ostringstream oss;
        oss << "Unable to create filter for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }

    Filter* stage = f(prevStage, options);
    return stage;
}


MultiFilter* StageFactory::createMultiFilter(const std::string& type, const std::vector<Stage*>& prevStages, const Options& options)
{
    MultiFilterCreator* f = getMultiFilterCreator(type);
    if (!f) {
        std::ostringstream oss;
        oss << "Unable to create multifilter for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }
    
    MultiFilter* stage = f(prevStages, options);
    return stage;
}


Writer* StageFactory::createWriter(const std::string& type, Stage& prevStage, const Options& options)
{
    WriterCreator* f = getWriterCreator(type);
    if (!f) {
        std::ostringstream oss;
        oss << "Unable to create writer for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }

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
    REGISTER_READER(FauxReader, pdal::drivers::faux::Reader);
    REGISTER_READER(LasReader, pdal::drivers::las::Reader);
#ifdef PDAL_HAVE_LIBLAS
    REGISTER_READER(LiblasReader, pdal::drivers::liblas::Reader);
#endif
#ifdef PDAL_HAVE_ORACLE
    REGISTER_READER(OciReader, pdal::drivers::oci::Reader);
#endif
    REGISTER_READER(PipelineReader, pdal::drivers::pipeline::Reader);
    REGISTER_READER(QfitReader, pdal::drivers::qfit::Reader);
    REGISTER_READER(TerrasolidReader, pdal::drivers::terrasolid::Reader);
}


void StageFactory::registerKnownFilters()
{
    REGISTER_FILTER(Attribute, pdal::filters::Attribute);
    REGISTER_FILTER(ByteSwap, pdal::filters::ByteSwap);
    REGISTER_FILTER(Cache, pdal::filters::Cache);
    REGISTER_FILTER(Chipper, pdal::filters::Chipper);
    REGISTER_FILTER(Color, pdal::filters::Color);
    REGISTER_FILTER(Crop, pdal::filters::Crop);
    REGISTER_FILTER(Decimation, pdal::filters::Decimation);
    REGISTER_FILTER(Descaling, pdal::filters::Descaling);
    REGISTER_FILTER(Reprojection, pdal::filters::Reprojection);
    REGISTER_FILTER(InPlaceReprojection, pdal::filters::InPlaceReprojection);
    REGISTER_FILTER(Scaling, pdal::filters::Scaling);
    REGISTER_FILTER(StatsFilter, pdal::filters::StatsFilter);
}


void StageFactory::registerKnownMultiFilters()
{   
    REGISTER_MULTIFILTER(MosaicFilter, pdal::filters::MosaicFilter);
}


void StageFactory::registerKnownWriters()
{
    REGISTER_WRITER(FauxWriter, pdal::drivers::faux::Writer);
    REGISTER_WRITER(LasWriter, pdal::drivers::las::Writer);
#ifdef PDAL_HAVE_LIBLAS
    REGISTER_WRITER(LiblasWriter, pdal::drivers::liblas::Writer);
#endif
#ifdef PDAL_HAVE_ORACLE
    REGISTER_WRITER(OciWriter, pdal::drivers::oci::Writer);
#endif
}


} // namespace pdal
