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
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>


#include <pdal/Drivers.hpp>
#include <pdal/Filters.hpp>

#include <pdal/Utils.hpp>


#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream>
#include <string>
#include <vector>
#include <stdio.h> // for funcptr

namespace pdal
{

//
// define the functions to create the readers
//
MAKE_READER_CREATOR(FauxReader, pdal::FauxReader)
MAKE_READER_CREATOR(LasReader, pdal::LasReader)
MAKE_READER_CREATOR(BpfReader, pdal::BpfReader)
MAKE_READER_CREATOR(BufferReader, pdal::BufferReader)
MAKE_READER_CREATOR(QfitReader, pdal::QfitReader)
MAKE_READER_CREATOR(SbetReader, pdal::SbetReader)
MAKE_READER_CREATOR(TerrasolidReader, pdal::TerrasolidReader)

//
// define the functions to create the filters
//
MAKE_FILTER_CREATOR(Chipper, pdal::ChipperFilter)
MAKE_FILTER_CREATOR(Colorization, pdal::ColorizationFilter)
MAKE_FILTER_CREATOR(Crop, pdal::CropFilter)
MAKE_FILTER_CREATOR(Decimation, pdal::DecimationFilter)
MAKE_FILTER_CREATOR(Ferry, pdal::FerryFilter)
MAKE_FILTER_CREATOR(Merge, pdal::MergeFilter)
MAKE_FILTER_CREATOR(MortonOrder, pdal::MortonOrderFilter)
MAKE_FILTER_CREATOR(Reprojection, pdal::ReprojectionFilter)
MAKE_FILTER_CREATOR(Sort, pdal::SortFilter)
MAKE_FILTER_CREATOR(Splitter, pdal::SplitterFilter)
MAKE_FILTER_CREATOR(Stats, pdal::StatsFilter)
MAKE_FILTER_CREATOR(Transformation, pdal::TransformationFilter)

//
// define the functions to create the writers
//
MAKE_WRITER_CREATOR(LasWriter, pdal::LasWriter)
MAKE_WRITER_CREATOR(SbetWriter, pdal::SbetWriter)
MAKE_WRITER_CREATOR(TextWriter, pdal::TextWriter)

StageFactory::StageFactory()
{
    registerKnownReaders();
    registerKnownFilters();
    registerKnownWriters();

    loadPlugins();
    return;
}


std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    StageFactory f;

    // filename may actually be a greyhound uri + pipelineId
    std::string http = filename.substr(0, 4);
    if (boost::iequals(http, "http") && f.getReaderCreator("readers.greyhound"))
        return "readers.greyhound";

    std::string ext = boost::filesystem::extension(filename);
    std::map<std::string, std::string> drivers;
    drivers["las"] = "readers.las";
    drivers["laz"] = "readers.las";
    drivers["bin"] = "readers.terrasolid";
    if (f.getReaderCreator("readers.greyhound"))
        drivers["greyhound"] = "readers.greyhound";
    drivers["qi"] = "readers.qfit";
    if (f.getReaderCreator("readers.nitf"))
    {
        drivers["nitf"] = "readers.nitf";
        drivers["ntf"] = "readers.nitf";
        drivers["nsf"] = "readers.nitf";
    }
    drivers["bpf"] = "readers.bpf";
    drivers["sbet"] = "readers.sbet";
    drivers["icebridge"] = "readers.icebridge";
    drivers["sqlite"] = "readers.sqlite";

    if (f.getReaderCreator("readers.rxp"))
        drivers["rxp"] = "readers.rxp";

    if (f.getReaderCreator("readers.pcd"))
        drivers["pcd"] = "readers.pcd";

    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);
    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


std::string StageFactory::inferWriterDriver(const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);

    boost::to_lower(ext);

    std::map<std::string, std::string> drivers;
    drivers["las"] = "writers.las";
    drivers["laz"] = "writers.las";
    StageFactory f;
    if (f.getWriterCreator("writers.pcd"))
        drivers["pcd"] = "writers.pcd";
    if (f.getWriterCreator("writers.pclvisualizer"))
        drivers["pclviz"] = "writers.pclvisualizer";
    drivers["sbet"] = "writers.sbet";
    drivers["csv"] = "writers.text";
    drivers["json"] = "writers.text";
    drivers["xyz"] = "writers.text";
    drivers["txt"] = "writers.text";
    if (f.getWriterCreator("writers.nitf"))
        drivers["ntf"] = "writers.nitf";
    drivers["sqlite"] = "writers.sqlite";

    if (boost::algorithm::iequals(filename, "STDOUT"))
    {
        return drivers["txt"];
    }

    if (ext == "") return drivers["txt"];
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return drivers["txt"];

    boost::to_lower(ext);
    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


pdal::Options StageFactory::inferWriterOptionsChanges(const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);
    boost::to_lower(ext);
    Options options;

    if (boost::algorithm::iequals(ext,".laz"))
    {
        options.add("compression", true);
    }

    StageFactory f;
    if (boost::algorithm::iequals(ext,".pcd") && f.getWriterCreator("writers.pcd"))
    {
        options.add("format","PCD");
    }

    options.add<std::string>("filename", filename);
    return options;
}


Reader* StageFactory::createReader(const std::string& type)
{
    ReaderCreator* f = getReaderCreator(type);
    if (!f)
    {
        std::ostringstream oss;
        oss << "Unable to create reader for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }
    Reader* stage = f();
    return stage;
}


Filter* StageFactory::createFilter(const std::string& type)
{
    FilterCreator* f = getFilterCreator(type);
    if (!f)
    {
        std::ostringstream oss;
        oss << "Unable to create filter for type '" << type << "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }

    return f();
}


Writer* StageFactory::createWriter(const std::string& type)
{
    WriterCreator* f = getWriterCreator(type);
    if (!f)
    {
        std::ostringstream oss;
        oss << "Unable to create writer for type '" << type <<
            "'. Does a driver with this type name exist?";
        throw pdal_error(oss.str());
    }

    return f();
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


void StageFactory::registerWriter(const std::string& type, WriterCreator* f)
{
    std::pair<std::string, WriterCreator*> p(type, f);
    m_writerCreators.insert(p);
}


void StageFactory::registerKnownReaders()
{
    REGISTER_READER(FauxReader, pdal::FauxReader);
    REGISTER_READER(BufferReader, pdal::BufferReader);
    REGISTER_READER(LasReader, pdal::LasReader);

    REGISTER_READER(QfitReader, pdal::QfitReader);
    REGISTER_READER(TerrasolidReader, pdal::TerrasolidReader);
    REGISTER_READER(BpfReader, pdal::BpfReader);
    REGISTER_READER(SbetReader, pdal::SbetReader);
}


void StageFactory::registerKnownFilters()
{
    REGISTER_FILTER(Chipper, pdal::ChipperFilter);
    REGISTER_FILTER(Colorization, pdal::ColorizationFilter);
    REGISTER_FILTER(Crop, pdal::CropFilter);
    REGISTER_FILTER(Decimation, pdal::DecimationFilter);
    REGISTER_FILTER(Ferry, pdal::FerryFilter);
    REGISTER_FILTER(Merge, pdal::MergeFilter);
    REGISTER_FILTER(MortonOrder, pdal::MortonOrderFilter);
    REGISTER_FILTER(Reprojection, pdal::ReprojectionFilter);
    REGISTER_FILTER(Sort, pdal::SortFilter);
    REGISTER_FILTER(Splitter, pdal::SplitterFilter);
    REGISTER_FILTER(Stats, pdal::StatsFilter);
    REGISTER_FILTER(Transformation, pdal::TransformationFilter);
}


void StageFactory::registerKnownWriters()
{
    REGISTER_WRITER(LasWriter, pdal::LasWriter);
    REGISTER_WRITER(SbetWriter, pdal::SbetWriter);
    REGISTER_WRITER(TextWriter, pdal::TextWriter);
}

void StageFactory::loadPlugins()
{
    using namespace boost::filesystem;

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color


    // If we don't have a driver path, we'll default to /usr/local/lib and lib

    if (pluginDir.size() == 0)
        pluginDir = "/usr/local/lib:./lib:../lib:../bin";

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir,
        boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        if (!boost::filesystem::is_directory(pluginPath))
            continue;
        directory_iterator dir(pluginPath), it, end;

        std::map<path, path> pluginFilenames;

        // Collect candidate filenames in the above form. Prefer symlink files
        // over hard files if their basenames are the same.
        for (it = dir; it != end; ++it)
        {
            path p = it->path();

            if (boost::algorithm::istarts_with(p.filename().string(),
                 "libpdal_plugin"))
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

            registerPlugin(filename.string());
            // std::string methodName = "PDALRegister_" + boost::algorithm::ireplace_first_copy(basename.string(), "libpdal_plugin_", "");
            // Utils::registerPlugin((void*)this, filename.string(), methodName);

        }
    }
}

void StageFactory::registerPlugin(std::string const& filename)
{
    using namespace boost::filesystem;
    path basename;

    path t = path(filename);
    for (; !t.extension().empty(); t = t.stem())
    {
        if (t.stem().extension().empty())
        {
            basename = t.stem().string();
        }
    }

    std::string base = basename.string();
    std::string pluginName = boost::algorithm::ireplace_first_copy(base, "libpdal_plugin_", "");

    std::string registerMethodName = "PDALRegister_" + pluginName;

    std::string versionMethodName = "PDALRegister_version_" + pluginName;

    Utils::registerPlugin((void*)this, filename, registerMethodName, versionMethodName);

}


std::map<std::string, pdal::StageInfo> const& StageFactory::getStageInfos() const
{
    return m_driver_info;
}

std::string StageFactory::toRST(std::string driverName) const
{
    std::ostringstream os;

    std::map<std::string, pdal::StageInfo> const& drivers = getStageInfos();
    typedef std::map<std::string, pdal::StageInfo>::const_iterator Iterator;

    Iterator i = drivers.find(driverName);
    std::string headline("------------------------------------------------------------------------------------------");

    os << headline << std::endl;
    os << "PDAL Options" << " (" << pdal::GetFullVersionString() << ")" <<std::endl;
    os << headline << std::endl << std::endl;

    // If we were given an explicit driver name, only display that.
    // Otherwise, display output for all of the registered drivers.
    if ( i != drivers.end())
    {
        os << i->second.optionsToRST() << std::endl;
    }
    else
    {
        for (i = drivers.begin(); i != drivers.end(); ++i)
        {
            os << i->second.optionsToRST() << std::endl;
        }
    }
    return os.str();
}

} // namespace pdal
