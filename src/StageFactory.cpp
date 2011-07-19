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

Reader* create_drivers_faux_reader(const Options& options) { return new pdal::drivers::faux::Reader(options); }

Reader* create_drivers_las_reader(const Options& options) { return new pdal::drivers::las::LasReader(options); }
Reader* create_drivers_liblas_reader(const Options& options) { return new pdal::drivers::liblas::LiblasReader(options); }
Reader* create_drivers_oci_reader(const Options& options) { return new pdal::drivers::oci::Reader(options); }
Reader* create_drivers_qfit_reader(const Options& options) { return new pdal::drivers::qfit::Reader(options); }
Reader* create_drivers_terrasolid_reader(const Options& options) { return new pdal::drivers::terrasolid::Reader(options); }

Writer* create_drivers_faux_writer(const DataStagePtr& prevStage, const Options& options) { return new pdal::drivers::faux::Writer(prevStage, options); }
Writer* create_drivers_las_writer(const DataStagePtr& prevStage, const Options& options) { return new pdal::drivers::las::LasWriter(prevStage, options); }
Writer* create_drivers_liblas_writer(const DataStagePtr& prevStage, const Options& options) { return new pdal::drivers::liblas::LiblasWriter(prevStage, options); }
Writer* create_drivers_oci_writer(const DataStagePtr& prevStage, const Options& options) { return new pdal::drivers::oci::Writer(prevStage, options); }

Filter* create_filters_byteswapfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::ByteSwapFilter(prevStage, options); }
Filter* create_filters_cachefilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::CacheFilter(prevStage, options); }
Filter* create_filters_chipper(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::Chipper(prevStage, options); }
Filter* create_filters_colorfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::ColorFilter(prevStage, options); }
Filter* create_filters_cropfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::CropFilter(prevStage, options); }
Filter* create_filters_decimationfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::DecimationFilter(prevStage, options); }
Filter* create_filters_reprojectionfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::ReprojectionFilter(prevStage, options); }
Filter* create_filters_scalingfilter(const DataStagePtr& prevStage, const Options& options) { return new pdal::filters::ScalingFilter(prevStage, options); }

MultiFilter* create_filters_mosaicfilter(const std::vector<const DataStagePtr>& prevStages, const Options& options) { return new pdal::filters::MosaicFilter(prevStages, options); }


StageFactory::StageFactory()
{
    registerKnownReaders();
    registerKnownFilters();
    registerKnownMultiFilters();
    registerKnownWriters();

    return;
}


ReaderPtr StageFactory::createReader(const std::string& type, const Options& options)
{
    ReaderCreator* f = getReaderCreator(type);
    Reader* stage = f(options);
    ReaderPtr ptr(stage);
    return ptr;
}


FilterPtr StageFactory::createFilter(const std::string& type, const DataStagePtr& prevStage, const Options& options)
{
    FilterCreator* f = getFilterCreator(type);
    Filter* stage = f(prevStage, options);
    FilterPtr ptr(stage);
    return ptr;
}


MultiFilterPtr StageFactory::createMultiFilter(const std::string& type, const std::vector<const DataStagePtr>& prevStages, const Options& options)
{
    MultiFilterCreator* f = getMultiFilterCreator(type);
    MultiFilter* stage = f(prevStages, options);
    MultiFilterPtr ptr(stage);
    return ptr;
}


WriterPtr StageFactory::createWriter(const std::string& type, const DataStagePtr& prevStage, const Options& options)
{
    WriterCreator* f = getWriterCreator(type);
    Writer* stage = f(prevStage, options);
    WriterPtr ptr(stage);
    return ptr;
}


template<typename T>
static T* findFirst(const std::string& type, std::map<std::string, T*> list)
{
    std::map<std::string, T*>::const_iterator iter = list.find(type);
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
    registerReader("drivers.faux.reader", create_drivers_faux_reader);
    registerReader("drivers.las.reader", create_drivers_las_reader);
    registerReader("drivers.liblas.reader", create_drivers_liblas_reader);
    registerReader("drivers.oci.reader", create_drivers_oci_reader);
    registerReader("drivers.qfit.reader", create_drivers_qfit_reader);
    registerReader("drivers.terrasolid.reader", create_drivers_terrasolid_reader);
}


void StageFactory::registerKnownFilters()
{
    registerFilter("filters.byteswap", create_filters_byteswapfilter);
    registerFilter("filters.cache", create_filters_cachefilter);
    registerFilter("filters.chipper", create_filters_chipper);
    registerFilter("filters.color", create_filters_colorfilter);
    registerFilter("filters.crop", create_filters_cropfilter);
    registerFilter("filters.decimation", create_filters_decimationfilter);
    registerFilter("filters.reprojection", create_filters_reprojectionfilter);
    registerFilter("filters.scaling", create_filters_scalingfilter);
}


void StageFactory::registerKnownMultiFilters()
{
    registerMultiFilter("filters.mosaic", create_filters_mosaicfilter);
}


void StageFactory::registerKnownWriters()
{
    registerWriter("drivers.faux.writer", create_drivers_faux_writer);
    registerWriter("drivers.las.writer", create_drivers_las_writer);
    registerWriter("drivers.liblas.writer", create_drivers_liblas_writer);
    registerWriter("drivers.oci.writer", create_drivers_oci_writer);
}


} // namespace pdal
