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
#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Reader.hpp>
#endif

#ifdef PDAL_HAVE_GDAL
#include <pdal/drivers/nitf/Reader.hpp>
#endif

#include <pdal/drivers/pipeline/Reader.hpp>
#include <pdal/drivers/qfit/Reader.hpp>
#include <pdal/drivers/terrasolid/Reader.hpp>

#include <pdal/drivers/faux/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/text/Writer.hpp>
#include <pdal/drivers/pcd/Writer.hpp>

#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Writer.hpp>
#endif

#ifdef PDAL_HAVE_P2G
#include <pdal/drivers/p2g/Writer.hpp>
#endif

#include <pdal/filters/ByteSwap.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/Color.hpp>
#include <pdal/filters/Colorization.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Decimation.hpp>
#include <pdal/filters/Index.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/filters/Reprojection.hpp>

#ifdef PDAL_HAVE_PYTHON
#include <pdal/filters/Predicate.hpp>
#include <pdal/filters/Programmable.hpp>
#endif

#include <pdal/filters/Scaling.hpp>
#include <pdal/filters/Selector.hpp>
#include <pdal/filters/Stats.hpp>

#include <pdal/filters/Mosaic.hpp>

#include <pdal/Utils.hpp>


#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <sstream>
#include <stdio.h> // for funcptr

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
#ifdef PDAL_HAVE_ORACLE
MAKE_READER_CREATOR(OciReader, pdal::drivers::oci::Reader)
#endif
#ifdef PDAL_HAVE_GDAL
MAKE_READER_CREATOR(NITFReader, pdal::drivers::nitf::Reader)
#endif

MAKE_READER_CREATOR(PipelineReader, pdal::drivers::pipeline::Reader)
MAKE_READER_CREATOR(QfitReader, pdal::drivers::qfit::Reader)
MAKE_READER_CREATOR(TerrasolidReader, pdal::drivers::terrasolid::Reader)

//
// define the functions to create the filters
//
MAKE_FILTER_CREATOR(ByteSwap, pdal::filters::ByteSwap)
MAKE_FILTER_CREATOR(Cache, pdal::filters::Cache)
MAKE_FILTER_CREATOR(Chipper, pdal::filters::Chipper)
MAKE_FILTER_CREATOR(Color, pdal::filters::Color)
MAKE_FILTER_CREATOR(Colorization, pdal::filters::Colorization)
MAKE_FILTER_CREATOR(Crop, pdal::filters::Crop)
MAKE_FILTER_CREATOR(Decimation, pdal::filters::Decimation)
MAKE_FILTER_CREATOR(Index, pdal::filters::Index)
MAKE_FILTER_CREATOR(InPlaceReprojection, pdal::filters::InPlaceReprojection)

#ifdef PDAL_HAVE_PYTHON
MAKE_FILTER_CREATOR(Predicate, pdal::filters::Predicate)
MAKE_FILTER_CREATOR(Programmable, pdal::filters::Programmable)
#endif

MAKE_FILTER_CREATOR(Reprojection, pdal::filters::Reprojection)
MAKE_FILTER_CREATOR(Scaling, pdal::filters::Scaling)
MAKE_FILTER_CREATOR(Selector, pdal::filters::Selector)
MAKE_FILTER_CREATOR(Stats, pdal::filters::Stats)

//
// define the functions to create the multifilters
//
MAKE_MULTIFILTER_CREATOR(Mosaic, pdal::filters::Mosaic)

//
// define the functions to create the writers
//
MAKE_WRITER_CREATOR(FauxWriter, pdal::drivers::faux::Writer)
MAKE_WRITER_CREATOR(LasWriter, pdal::drivers::las::Writer)
MAKE_WRITER_CREATOR(TextWriter, pdal::drivers::text::Writer)
MAKE_WRITER_CREATOR(PCDWriter, pdal::drivers::pcd::Writer)
#ifdef PDAL_HAVE_ORACLE
MAKE_WRITER_CREATOR(OciWriter, pdal::drivers::oci::Writer)
#endif

#ifdef PDAL_HAVE_P2G
MAKE_WRITER_CREATOR(P2GWriter, pdal::drivers::p2g::Writer)
#endif


StageFactory::StageFactory()
{
    registerKnownReaders();
    registerKnownFilters();
    registerKnownMultiFilters();
    registerKnownWriters();

    loadPlugins();
    return;
}


Reader* StageFactory::createReader(const std::string& type, const Options& options)
{
    ReaderCreator* f = getReaderCreator(type);
    if (!f)
    {
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
    if (!f)
    {
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
    if (!f)
    {
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
    if (!f)
    {
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
#ifdef PDAL_HAVE_ORACLE
    REGISTER_READER(OciReader, pdal::drivers::oci::Reader);
#endif
#ifdef PDAL_HAVE_GDAL
    REGISTER_READER(NITFReader, pdal::drivers::nitf::Reader);
#endif

    REGISTER_READER(PipelineReader, pdal::drivers::pipeline::Reader);
    REGISTER_READER(QfitReader, pdal::drivers::qfit::Reader);
    REGISTER_READER(TerrasolidReader, pdal::drivers::terrasolid::Reader);
}


void StageFactory::registerKnownFilters()
{
    REGISTER_FILTER(ByteSwap, pdal::filters::ByteSwap);
    REGISTER_FILTER(Cache, pdal::filters::Cache);
    REGISTER_FILTER(Chipper, pdal::filters::Chipper);
    REGISTER_FILTER(Color, pdal::filters::Color);
    REGISTER_FILTER(Colorization, pdal::filters::Colorization);
    REGISTER_FILTER(Crop, pdal::filters::Crop);
    REGISTER_FILTER(Decimation, pdal::filters::Decimation);
    REGISTER_FILTER(Reprojection, pdal::filters::Reprojection);
    REGISTER_FILTER(Index, pdal::filters::Index);
    REGISTER_FILTER(InPlaceReprojection, pdal::filters::InPlaceReprojection);

#ifdef PDAL_HAVE_PYTHON
    REGISTER_FILTER(Predicate, pdal::filters::Predicate);
    REGISTER_FILTER(Programmable, pdal::filters::Programmable);
#endif

    REGISTER_FILTER(Reprojection, pdal::filters::Reprojection);
    REGISTER_FILTER(Scaling, pdal::filters::Scaling);
    REGISTER_FILTER(Selector, pdal::filters::Selector);
    REGISTER_FILTER(Stats, pdal::filters::Stats);
}


void StageFactory::registerKnownMultiFilters()
{
    REGISTER_MULTIFILTER(Mosaic, pdal::filters::Mosaic);
}


void StageFactory::registerKnownWriters()
{
    REGISTER_WRITER(FauxWriter, pdal::drivers::faux::Writer);
    REGISTER_WRITER(LasWriter, pdal::drivers::las::Writer);
    REGISTER_WRITER(TextWriter, pdal::drivers::text::Writer);
    REGISTER_WRITER(PCDWriter, pdal::drivers::pcd::Writer);
#ifdef PDAL_HAVE_ORACLE
    REGISTER_WRITER(OciWriter, pdal::drivers::oci::Writer);
#endif

#ifdef PDAL_HAVE_P2G
    REGISTER_WRITER(P2GWriter, pdal::drivers::p2g::Writer);
#endif

}

void StageFactory::loadPlugins()
{
    using namespace boost::filesystem;

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color


    // If we don't have a driver path, we're not loading anything
    // FIXME: support setting the plugin name directly from the
    // PipelineReader

    if (pluginDir.size() == 0) return;

    directory_iterator dir(pluginDir), it, end;

    std::map<path, path> pluginFilenames;

    // Collect candidate filenames in the above form. Prefer symlink files
    // over hard files if their basenames are the same.
    for (it = dir; it != end; ++it)
    {
        path p = it->path();

        if (boost::algorithm::istarts_with(p.filename().string(), "libpdal_plugin"))
        {
            path extension = p.extension();
            if (boost::algorithm::iends_with(extension.string(), "DLL") ||
                    boost::algorithm::iends_with(extension.string(), "DYLIB") ||
                    boost::algorithm::iends_with(extension.string(), "SO"))
            {
                std::string basename;

                // Step through the stems until the extension of the stem
                // is empty. This is our basename.  For example,
                // libpdal_plugin_writer_text.0.dylib will basename down to
                // libpdal_plugin_writer_text and so will
                // libpdal_plugin_writer_text.dylib
                // copy the path so we can modify in place
                path t = p;
                for (; !t.extension().empty(); t = t.stem())
                {
                    if (t.stem().extension().empty())
                    {
                        basename = t.stem().string();
                    }
                }

                if (pluginFilenames.find(basename) == pluginFilenames.end())
                {
                    // We haven't already loaded a plugin with this basename,
                    // load it.
                    pluginFilenames.insert(std::pair<path, path>(basename, p));
                }
                else
                {
                    // We already have a filename with the basename of this
                    // file.  If the basename of our current file is a symlink
                    // we're going to replace what's in the map with ours because
                    // we are going to presume that a symlink'd file is more
                    // cannonical than a hard file of the same name.
                    std::map<path, path>::iterator i = pluginFilenames.find(basename);
                    if (it->symlink_status().type() == symlink_file)
                    {
                        // Take the symlink over a hard SO
                        i->second = p;
                    }
                }
            }
        }
    }

    std::map<std::string, std::string> registerMethods;

    for (std::map<path, path>::iterator t = pluginFilenames.begin();
            t!= pluginFilenames.end(); t ++)
    {
        // Basenames must be in the following form:
        // libpdal_plugin_writer_text or libpdal_plugin_filter_color
        // The last two tokens are the stage type and the stage name.
        path basename = t->first;
        path filename = t->second;

        void* pRegister;

        std::string methodName = "PDALRegister_" + boost::algorithm::ireplace_first_copy(basename.string(), "libpdal_plugin_", "");

        // std::cout << "Loading: " << methodName << " from dll "<< t->first << " with path: " << t->second <<std::endl;

        pRegister = Utils::getDLLSymbol(filename.string(), methodName);
        if (pRegister != NULL)
        {
            ((void (*)(void*)) pRegister)(this);
        }

    }
}


} // namespace pdal
