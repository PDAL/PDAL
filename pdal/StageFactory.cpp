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

#include <filters/ApproximateCoplanarFilter.hpp>
#include <filters/AssignFilter.hpp>
#include <filters/ChipperFilter.hpp>
#include <filters/ClusterFilter.hpp>
#include <filters/ColorizationFilter.hpp>
#include <filters/ColorinterpFilter.hpp>
#include <filters/ComputeRangeFilter.hpp>
#include <filters/CropFilter.hpp>
#include <filters/DecimationFilter.hpp>
#include <filters/DividerFilter.hpp>
#include <filters/EigenvaluesFilter.hpp>
#include <filters/ELMFilter.hpp>
#include <filters/EstimateRankFilter.hpp>
#include <filters/FerryFilter.hpp>
#include <filters/GreedyProjection.hpp>
#include <filters/GroupByFilter.hpp>
#include <filters/HAGFilter.hpp>
#include <filters/HeadFilter.hpp>
#include <filters/IQRFilter.hpp>
#include <filters/KDistanceFilter.hpp>
#include <filters/LocateFilter.hpp>
#include <filters/LOFFilter.hpp>
#include <filters/MADFilter.hpp>
#include <filters/MergeFilter.hpp>
#include <filters/MongusFilter.hpp>
#include <filters/MortonOrderFilter.hpp>
#include <filters/NormalFilter.hpp>
#include <filters/OutlierFilter.hpp>
#include <filters/OverlayFilter.hpp>
#include <filters/PMFFilter.hpp>
#include <filters/PoissonFilter.hpp>
#include <filters/RadialDensityFilter.hpp>
#include <filters/RandomizeFilter.hpp>
#include <filters/RangeFilter.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <filters/SampleFilter.hpp>
#include <filters/SMRFilter.hpp>
#include <filters/SortFilter.hpp>
#include <filters/SplitterFilter.hpp>
#include <filters/StatsFilter.hpp>
#include <filters/TailFilter.hpp>
#include <filters/TransformationFilter.hpp>
#include <filters/VoxelCenterNearestNeighborFilter.hpp>
#include <filters/VoxelCentroidNearestNeighborFilter.hpp>

#include <kernels/DeltaKernel.hpp>
#include <kernels/DiffKernel.hpp>
#include <kernels/GroundKernel.hpp>
#include <kernels/HausdorffKernel.hpp>
#include <kernels/InfoKernel.hpp>
#include <kernels/MergeKernel.hpp>
#include <kernels/PipelineKernel.hpp>
#include <kernels/RandomKernel.hpp>
#include <kernels/SortKernel.hpp>
#include <kernels/SplitKernel.hpp>
#include <kernels/TIndexKernel.hpp>
#include <kernels/TranslateKernel.hpp>

// readers
#include <io/BpfReader.hpp>
#include <io/FauxReader.hpp>
#include <io/GDALReader.hpp>
#include <io/Ilvis2Reader.hpp>
#include <io/LasReader.hpp>
#include <io/OptechReader.hpp>
#include <io/PlyReader.hpp>
#include <io/PtsReader.hpp>
#include <io/QfitReader.hpp>
#include <io/SbetReader.hpp>
#include <io/TerrasolidReader.hpp>
#include <io/TextReader.hpp>
#include <io/TIndexReader.hpp>

// writers
#include <io/BpfWriter.hpp>
#include <io/GDALWriter.hpp>
#include <io/LasWriter.hpp>
#include <io/OGRWriter.hpp>
#include <io/PlyWriter.hpp>
#include <io/SbetWriter.hpp>
#include <io/TextWriter.hpp>
#include <io/NullWriter.hpp>

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
        { "readers.sqlite", { "sqlite", "gpkg" } },
        { "readers.matlab", { "mat" } },
        { "readers.mrsid", { "sid" } },
        { "readers.tindex", { "tindex" } },
        { "readers.text", { "csv", "txt" } },
        { "readers.icebridge", { "h5" } },

        { "writers.bpf", { "bpf" } },
        { "writers.text", { "csv", "json", "txt", "xyz" } },
        { "writers.las", { "las", "laz" } },
        { "writers.matlab", { "mat" } },
        { "writers.nitf", { "nitf", "nsf", "ntf" } },
        { "writers.pcd", { "pcd" } },
        { "writers.ply", { "ply" } },
        { "writers.sbet", { "sbet" } },
        { "writers.derivative", { "derivative" } },
        { "writers.sqlite", { "sqlite", "gpkg" } },
        { "writers.gdal", { "tif", "tiff", "vrt" } },
        { "writers.ogr", { "shp", "geojson" } },
        { "writers.greyhound", { "greyhound" } },
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
        { "csv", "readers.text" },
        { "greyhound", "readers.greyhound" },
        { "gpkg", "readers.sqlite" },
        { "icebridge", "readers.icebridge" },
        { "las", "readers.las" },
        { "laz", "readers.las" },
        { "mat", "readers.matlab" },
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
        { "txt", "readers.text" },
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

    static const std::string ghPrefix("greyhound://");

    if (filename == "STDOUT")
        ext = ".txt";
    else if (Utils::iequals(filename.substr(0, ghPrefix.size()), ghPrefix))
        ext = ".greyhound";      // Make it look like an extension.
    else
        ext = Utils::tolower(FileUtils::extension(filename));

    static std::map<std::string, std::string> drivers =
    {
        { "bpf", "writers.bpf" },
        { "csv", "writers.text" },
        { "json", "writers.text" },
        { "las", "writers.las" },
        { "laz", "writers.las" },
        { "gpkg", "writers.sqlite" },
        { "greyhound", "writers.greyhound" },
        { "mat", "writers.matlab" },
        { "ntf", "writers.nitf" },
        { "pcd", "writers.pcd" },
        { "ply", "writers.ply" },
        { "sbet", "writers.sbet" },
        { "derivative", "writers.derivative" },
        { "sqlite", "writers.sqlite" },
        { "txt", "writers.text" },
        { "xyz", "writers.text" },
        { "", "writers.text" },
        { "tif", "writers.gdal" },
        { "tiff", "writers.gdal" },
        { "vrt", "writers.gdal" },
        { "shp", "writers.ogr" },
        { "geojson", "writers.ogr" }
    };

    // Strip off '.' and make lowercase.
    if (ext.length())
        ext = Utils::tolower(ext.substr(1, ext.length() - 1));

    return drivers[ext];
}


StageFactory::StageFactory(bool ignored)
{
    (void)ignored;

    // filters
    ApproximateCoplanarFilter_InitPlugin();
    AssignFilter_InitPlugin();
    ChipperFilter_InitPlugin();
    ClusterFilter_InitPlugin();
    ColorizationFilter_InitPlugin();
    ColorinterpFilter_InitPlugin();
    ComputeRangeFilter_InitPlugin();
    CropFilter_InitPlugin();
    DecimationFilter_InitPlugin();
    DividerFilter_InitPlugin();
    EigenvaluesFilter_InitPlugin();
    ELMFilter_InitPlugin();
    EstimateRankFilter_InitPlugin();
    FerryFilter_InitPlugin();
    GreedyProjection_InitPlugin();
    GroupByFilter_InitPlugin();
    HAGFilter_InitPlugin();
    HeadFilter_InitPlugin();
    IQRFilter_InitPlugin();
    KDistanceFilter_InitPlugin();
    LocateFilter_InitPlugin();
    LOFFilter_InitPlugin();
    MADFilter_InitPlugin();
    MergeFilter_InitPlugin();
    MongusFilter_InitPlugin();
    MortonOrderFilter_InitPlugin();
    NormalFilter_InitPlugin();
    OutlierFilter_InitPlugin();
    OverlayFilter_InitPlugin();
    PMFFilter_InitPlugin();
    PoissonFilter_InitPlugin();
    RadialDensityFilter_InitPlugin();
    RandomizeFilter_InitPlugin();
    RangeFilter_InitPlugin();
    ReprojectionFilter_InitPlugin();
    SampleFilter_InitPlugin();
    SMRFilter_InitPlugin();
    SortFilter_InitPlugin();
    SplitterFilter_InitPlugin();
    StatsFilter_InitPlugin();
    TailFilter_InitPlugin();
    TransformationFilter_InitPlugin();
    VoxelCenterNearestNeighborFilter_InitPlugin();
    VoxelCentroidNearestNeighborFilter_InitPlugin();

    // kernels
    DeltaKernel_InitPlugin();
    DiffKernel_InitPlugin();
    GroundKernel_InitPlugin();
    HausdorffKernel_InitPlugin();
    InfoKernel_InitPlugin();
    MergeKernel_InitPlugin();
    PipelineKernel_InitPlugin();
    RandomKernel_InitPlugin();
    SortKernel_InitPlugin();
    SplitKernel_InitPlugin();
    TIndexKernel_InitPlugin();
    TranslateKernel_InitPlugin();

    // readers
    BpfReader_InitPlugin();
    FauxReader_InitPlugin();
    GDALReader_InitPlugin();
    Ilvis2Reader_InitPlugin();
    LasReader_InitPlugin();
    OptechReader_InitPlugin();
    PlyReader_InitPlugin();
    PtsReader_InitPlugin();
    QfitReader_InitPlugin();
    SbetReader_InitPlugin();
    TerrasolidReader_InitPlugin();
    TextReader_InitPlugin();
    TIndexReader_InitPlugin();

    // writers
    BpfWriter_InitPlugin();
    GDALWriter_InitPlugin();
    LasWriter_InitPlugin();
    OGRWriter_InitPlugin();
    PlyWriter_InitPlugin();
    SbetWriter_InitPlugin();
    TextWriter_InitPlugin();
    NullWriter_InitPlugin();
}


Stage *StageFactory::createStage(std::string const& stage_name)
{
    static_assert(0 < sizeof(Stage), "");
    Stage *s = PluginManager<Stage>::createObject(stage_name);
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
