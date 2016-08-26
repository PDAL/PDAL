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
#include <pdal/PluginManager.hpp>
#include <pdal/util/FileUtils.hpp>

// filters
#include <approximatecoplanar/ApproximateCoplanarFilter.hpp>
#include <attribute/AttributeFilter.hpp>
#include <chipper/ChipperFilter.hpp>
#include <colorization/ColorizationFilter.hpp>
#include <crop/CropFilter.hpp>
#include <decimation/DecimationFilter.hpp>
#include <divider/DividerFilter.hpp>
#include <eigenvalues/EigenvaluesFilter.hpp>
#include <estimaterank/EstimateRankFilter.hpp>
#include <ferry/FerryFilter.hpp>
#include <hag/HAGFilter.hpp>
#include <merge/MergeFilter.hpp>
#include <mongus/MongusFilter.hpp>
#include <mortonorder/MortonOrderFilter.hpp>
#include <normal/NormalFilter.hpp>
#include <outlier/OutlierFilter.hpp>
#include <pmf/PMFFilter.hpp>
#include <range/RangeFilter.hpp>
#include <reprojection/ReprojectionFilter.hpp>
#include <sample/SampleFilter.hpp>
#include <smrf/SMRFilter.hpp>
#include <sort/SortFilter.hpp>
#include <splitter/SplitterFilter.hpp>
#include <stats/StatsFilter.hpp>
#include <transformation/TransformationFilter.hpp>

// readers
#include <bpf/BpfReader.hpp>
#include <faux/FauxReader.hpp>
#include <gdal/GDALReader.hpp>
#include <ilvis2/Ilvis2Reader.hpp>
#include <las/LasReader.hpp>
#include <optech/OptechReader.hpp>
#include <buffer/BufferReader.hpp>
#include <ply/PlyReader.hpp>
#include <pts/PtsReader.hpp>
#include <qfit/QfitReader.hpp>
#include <sbet/SbetReader.hpp>
#include <terrasolid/TerrasolidReader.hpp>
#include <text/TextReader.hpp>
#include <tindex/TIndexReader.hpp>

// writers
#include <bpf/BpfWriter.hpp>
#include <las/LasWriter.hpp>
#include <ply/PlyWriter.hpp>
#include <sbet/SbetWriter.hpp>
#include <derivative/DerivativeWriter.hpp>
#include <text/TextWriter.hpp>
#include <null/NullWriter.hpp>

#include <sstream>
#include <string>
#include <stdio.h> // for funcptr

namespace pdal
{

StringList StageFactory::extensions(const std::string& driver)
{
    static std::map<std::string, StringList> exts =
    {
        { "readers.terrasolid", { "bin" } },
        { "readers.bpf", { "bpf" }  },
        { "readers.optech", { "csd" } },
        { "readers.greyhound", { "greyhound" } },
        { "readers.icebridge", { "icebridge" } },
        { "readers.las", { "las", "laz" } },
        { "readers.nitf", { "nitf", "nsf", "ntf" } },
        { "readers.pcd", { "pcd" } },
        { "readers.ply", { "ply" } },
        { "readers.pts", { "pts" } },
        { "readers.qfit", { "qi" } },
        { "readers.rxp", { "rxp" } },
        { "readers.sbet", { "sbet" } },
        { "readers.sqlite", { "sqlite" } },
        { "readers.mrsid", { "sid" } },
        { "readers.tindex", { "tindex" } },
        { "readers.txt", { "txt" } },
        { "readers.icebridge", { "h5" } },

        { "writers.bpf", { "bpf" } },
        { "writers.text", { "csv", "json", "txt", "xyz" } },
        { "writers.las", { "las", "laz" } },
        { "writers.matlab", { "mat" } },
        { "writers.nitf", { "nitf", "nsf", "ntf" } },
        { "writers.pcd", { "pcd" } },
        { "writers.pclvisualizer", { "pclvis" } },
        { "writers.ply", { "ply" } },
        { "writers.sbet", { "sbet" } },
        { "writers.derivative", { "derivative" } },
        { "writers.sqlite", { "sqlite" } },
    };

    return exts[driver];
}

std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    static std::map<std::string, std::string> drivers =
    {
        { "bin", "readers.terrasolid" },
        { "bpf", "readers.bpf" },
        { "csd", "readers.optech" },
        { "greyhound", "readers.greyhound" },
        { "icebridge", "readers.icebridge" },
        { "las", "readers.las" },
        { "laz", "readers.las" },
        { "nitf", "readers.nitf" },
        { "nsf", "readers.nitf" },
        { "ntf", "readers.nitf" },
        { "pcd", "readers.pcd" },
        { "ply", "readers.ply" },
        { "pts", "readers.pts" },
        { "qi", "readers.qfit" },
        { "rxp", "readers.rxp" },
        { "sbet", "readers.sbet" },
        { "sqlite", "readers.sqlite" },
        { "sid", "readers.mrsid" },
        { "tindex", "readers.tindex" },
        { "txt", "readers.txt" },
        { "h5", "readers.icebridge" }
    };

    static const std::string ghPrefix("greyhound://");

    std::string ext;
    // filename may actually be a greyhound uri + pipelineId
    if (Utils::iequals(filename.substr(0, ghPrefix.size()), ghPrefix))
        ext = ".greyhound";      // Make it look like an extension.
    else
        ext = FileUtils::extension(filename);

    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1, ext.length() - 1));

    return drivers[ext]; // will be "" if not found
}


std::string StageFactory::inferWriterDriver(const std::string& filename)
{
    std::string ext;

    if (filename == "STDOUT")
        ext = ".txt";
    else
        ext = Utils::tolower(FileUtils::extension(filename));

    static std::map<std::string, std::string> drivers =
    {
        { "bpf", "writers.bpf" },
        { "csv", "writers.text" },
        { "json", "writers.text" },
        { "las", "writers.las" },
        { "laz", "writers.las" },
        { "mat", "writers.matlab" },
        { "ntf", "writers.nitf" },
        { "pcd", "writers.pcd" },
        { "pclviz", "writers.pclvisualizer" },
        { "ply", "writers.ply" },
        { "sbet", "writers.sbet" },
        { "derivative", "writers.derivative" },
        { "sqlite", "writers.sqlite" },
        { "txt", "writers.text" },
        { "xyz", "writers.text" },
        { "", "writers.text" }
    };

    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1, ext.length() - 1));

    return drivers[ext];
}


StageFactory::StageFactory(bool no_plugins)
{
    if (!no_plugins)
    {
        PluginManager::loadAll(PF_PluginType_Filter | PF_PluginType_Reader |
            PF_PluginType_Writer);
    }

    // filters
    PluginManager::initializePlugin(ApproximateCoplanarFilter_InitPlugin);
    PluginManager::initializePlugin(AttributeFilter_InitPlugin);
    PluginManager::initializePlugin(ChipperFilter_InitPlugin);
    PluginManager::initializePlugin(ColorizationFilter_InitPlugin);
    PluginManager::initializePlugin(CropFilter_InitPlugin);
    PluginManager::initializePlugin(DecimationFilter_InitPlugin);
    PluginManager::initializePlugin(DividerFilter_InitPlugin);
    PluginManager::initializePlugin(EigenvaluesFilter_InitPlugin);
    PluginManager::initializePlugin(EstimateRankFilter_InitPlugin);
    PluginManager::initializePlugin(FerryFilter_InitPlugin);
    PluginManager::initializePlugin(HAGFilter_InitPlugin);
    PluginManager::initializePlugin(MergeFilter_InitPlugin);
    PluginManager::initializePlugin(MongusFilter_InitPlugin);
    PluginManager::initializePlugin(MortonOrderFilter_InitPlugin);
    PluginManager::initializePlugin(NormalFilter_InitPlugin);
    PluginManager::initializePlugin(OutlierFilter_InitPlugin);
    PluginManager::initializePlugin(PMFFilter_InitPlugin);
    PluginManager::initializePlugin(RangeFilter_InitPlugin);
    PluginManager::initializePlugin(ReprojectionFilter_InitPlugin);
    PluginManager::initializePlugin(SampleFilter_InitPlugin);
    PluginManager::initializePlugin(SMRFilter_InitPlugin);
    PluginManager::initializePlugin(SortFilter_InitPlugin);
    PluginManager::initializePlugin(SplitterFilter_InitPlugin);
    PluginManager::initializePlugin(StatsFilter_InitPlugin);
    PluginManager::initializePlugin(TransformationFilter_InitPlugin);

    // readers
    PluginManager::initializePlugin(BpfReader_InitPlugin);
    PluginManager::initializePlugin(FauxReader_InitPlugin);
    PluginManager::initializePlugin(GDALReader_InitPlugin);
    PluginManager::initializePlugin(Ilvis2Reader_InitPlugin);
    PluginManager::initializePlugin(LasReader_InitPlugin);
    PluginManager::initializePlugin(OptechReader_InitPlugin);
    PluginManager::initializePlugin(PlyReader_InitPlugin);
    PluginManager::initializePlugin(PtsReader_InitPlugin);
    PluginManager::initializePlugin(QfitReader_InitPlugin);
    PluginManager::initializePlugin(SbetReader_InitPlugin);
    PluginManager::initializePlugin(TerrasolidReader_InitPlugin);
    PluginManager::initializePlugin(TextReader_InitPlugin);
    PluginManager::initializePlugin(TIndexReader_InitPlugin);

    // writers
    PluginManager::initializePlugin(BpfWriter_InitPlugin);
    PluginManager::initializePlugin(LasWriter_InitPlugin);
    PluginManager::initializePlugin(PlyWriter_InitPlugin);
    PluginManager::initializePlugin(SbetWriter_InitPlugin);
    PluginManager::initializePlugin(DerivativeWriter_InitPlugin);
    PluginManager::initializePlugin(TextWriter_InitPlugin);
    PluginManager::initializePlugin(NullWriter_InitPlugin);
}


Stage *StageFactory::createStage(std::string const& stage_name)
{
    static_assert(0 < sizeof(Stage), "");
    Stage *s = static_cast<Stage*>(PluginManager::createObject(stage_name));
    if (s)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_ownedStages.push_back(std::unique_ptr<Stage>(s));
    }
    return s;
}


void StageFactory::destroyStage(Stage *s)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto it = m_ownedStages.begin(); it != m_ownedStages.end(); ++it)
    {
        if (s == it->get())
        {
            m_ownedStages.erase(it);
            break;
        }
    }
}

} // namespace pdal
