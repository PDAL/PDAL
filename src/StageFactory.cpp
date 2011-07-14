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
    
static Reader* create_drivers_faux_reader(const Options& options)        { return new pdal::drivers::faux::Reader(options); }
static Reader* create_drivers_las_reader(const Options& options)         { return new pdal::drivers::las::LasReader(options); }
static Reader* create_drivers_liblas_reader(const Options& options)      { return new pdal::drivers::liblas::LiblasReader(options); }
static Reader* create_drivers_oci_reader(const Options& options)         { return new pdal::drivers::oci::Reader(options); }
static Reader* create_drivers_qfit_reader(const Options& options)        { return new pdal::drivers::qfit::Reader(options); }
static Reader* create_drivers_terrasolid_reader(const Options& options)  { return new pdal::drivers::terrasolid::Reader(options); }

static Writer* create_drivers_faux_writer(const Stage& prevStage, const Options& options)    { return new pdal::drivers::faux::Writer(prevStage, options); }
static Writer* create_drivers_las_writer(const Stage& prevStage, const Options& options)     { return new pdal::drivers::las::LasWriter(prevStage, options); }
static Writer* create_drivers_liblas_writer(const Stage& prevStage, const Options& options)  { return new pdal::drivers::liblas::LiblasWriter(prevStage, options); }
static Writer* create_drivers_oci_writer(const Stage& prevStage, const Options& options)     { return new pdal::drivers::oci::Writer(prevStage, options); }

static Filter* create_filters_byteswapfilter(const Stage& prevStage, const Options& options)      { return new pdal::filters::ByteSwapFilter(prevStage, options); }
static Filter* create_filters_cachefilter(const Stage& prevStage, const Options& options)         { return new pdal::filters::CacheFilter(prevStage, options); }
static Filter* create_filters_chipper(const Stage& prevStage, const Options& options)             { return new pdal::filters::Chipper(prevStage, options); }
static Filter* create_filters_colorfilter(const Stage& prevStage, const Options& options)         { return new pdal::filters::ColorFilter(prevStage, options); }
static Filter* create_filters_cropfilter(const Stage& prevStage, const Options& options)          { return new pdal::filters::CropFilter(prevStage, options); }
static Filter* create_filters_decimationfilter(const Stage& prevStage, const Options& options)    { return new pdal::filters::DecimationFilter(prevStage, options); }
static Filter* create_filters_reprojectionfilter(const Stage& prevStage, const Options& options)  { return new pdal::filters::ReprojectionFilter(prevStage, options); }
static Filter* create_filters_scalingfilter(const Stage& prevStage, const Options& options)       { return new pdal::filters::ScalingFilter(prevStage, options); }

static MultiFilter* create_filters_mosaicfilter(const std::vector<const Stage*>& prevStages, const Options& options)  { return new pdal::filters::MosaicFilter(prevStages, options); }


StageFactory::StageFactory()
{
    registerKnownStages();

    return;
}


boost::shared_ptr<Reader> StageFactory::createReader(const std::string& type, const Options& options)
{
    ReaderCreator* f = getReaderCreator(type);
    Reader* stage = f(options);
    boost::shared_ptr<Reader> ptr(stage);
    return ptr;
}


boost::shared_ptr<Filter> StageFactory::createFilter(const std::string& type, const Stage& prevStage, const Options& options)
{
    FilterCreator* f = getFilterCreator(type);
    Filter* stage = f(prevStage, options);
    boost::shared_ptr<Filter> ptr(stage);
    return ptr;
}


boost::shared_ptr<MultiFilter> StageFactory::createMultiFilter(const std::string& type, const std::vector<const Stage*>& prevStages, const Options& options)
{
    MultiFilterCreator* f = getMultiFilterCreator(type);
    MultiFilter* stage = f(prevStages, options);
    boost::shared_ptr<MultiFilter> ptr(stage);
    return ptr;
}


boost::shared_ptr<Writer> StageFactory::createWriter(const std::string& type, const Stage& prevStage, const Options& options)
{
    WriterCreator* f = getWriterCreator(type);
    Writer* stage = f(prevStage, options);
    boost::shared_ptr<Writer> ptr(stage);
    return ptr;
}


StageFactory::ReaderCreator* StageFactory::getReaderCreator(const std::string& type)
{
    ReaderCreatorList::const_iterator iter = m_readerCreators.find(type);
    if (iter == m_readerCreators.end()) throw pdal_error("unknown reader stage type");
    ReaderCreator* f = iter->second;
    return f;
}


StageFactory::FilterCreator* StageFactory::getFilterCreator(const std::string& type)
{
    FilterCreatorList::const_iterator iter = m_filterCreators.find(type);
    if (iter == m_filterCreators.end()) throw pdal_error("unknown filter stage type");
    FilterCreator* f = iter->second;
    return f;
}


StageFactory::MultiFilterCreator* StageFactory::getMultiFilterCreator(const std::string& type)
{
    MultiFilterCreatorList::const_iterator iter = m_multifilterCreators.find(type);
    if (iter == m_multifilterCreators.end()) throw pdal_error("unknown multifilter stage type");
    MultiFilterCreator* f = iter->second;
    return f;
}


StageFactory::WriterCreator* StageFactory::getWriterCreator(const std::string& type)
{
    WriterCreatorList::const_iterator iter = m_writerCreators.find(type);
    if (iter == m_writerCreators.end()) throw pdal_error("unknown writer stage type");
    WriterCreator* f = iter->second;
    return f;
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


void StageFactory::registerKnownStages()
{
    registerReader("drivers.faux.reader", create_drivers_faux_reader);
    registerReader("drivers.las.reader", create_drivers_las_reader);
    registerReader("drivers.liblas.reader", create_drivers_liblas_reader);
    registerReader("drivers.oci.reader", create_drivers_oci_reader);
    registerReader("drivers.qfit.reader", create_drivers_qfit_reader);
    registerReader("drivers.terrasolid.reader", create_drivers_terrasolid_reader);

    registerWriter("drivers.faux.writer", create_drivers_faux_writer);
    registerWriter("drivers.las.writer", create_drivers_las_writer);
    registerWriter("drivers.liblas.writer", create_drivers_liblas_writer);
    registerWriter("drivers.oci.writer", create_drivers_oci_writer);

    registerFilter("filters.byteswap", create_filters_byteswapfilter);
    registerFilter("filters.cache", create_filters_cachefilter);
    registerFilter("filters.chipper", create_filters_chipper);
    registerFilter("filters.color", create_filters_colorfilter);
    registerFilter("filters.crop", create_filters_cropfilter);
    registerFilter("filters.decimation", create_filters_decimationfilter);
    registerFilter("filters.reprojection", create_filters_reprojectionfilter);
    registerFilter("filters.scaling", create_filters_scalingfilter);

    registerMultiFilter("filters.mosaicfilter", create_filters_mosaicfilter);

    return;
}


} // namespace pdal
