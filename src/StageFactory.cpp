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

// filters
#include <chipper/ChipperFilter.hpp>
#include <colorization/ColorizationFilter.hpp>
#include <crop/CropFilter.hpp>
#include <decimation/DecimationFilter.hpp>
#include <ferry/FerryFilter.hpp>
#include <merge/MergeFilter.hpp>
#include <mortonorder/MortonOrderFilter.hpp>
#include <range/RangeFilter.hpp>
#include <reprojection/ReprojectionFilter.hpp>
#include <sort/SortFilter.hpp>
#include <splitter/SplitterFilter.hpp>
#include <stats/StatsFilter.hpp>
#include <transformation/TransformationFilter.hpp>

// readers
#include <bpf/BpfReader.hpp>
#include <faux/FauxReader.hpp>
#include <las/LasReader.hpp>
#include <optech/OptechReader.hpp>
#include <pdal/BufferReader.hpp>
#include <ply/PlyReader.hpp>
#include <qfit/QfitReader.hpp>
#include <sbet/SbetReader.hpp>
#include <terrasolid/TerrasolidReader.hpp>

// writers
#include <bpf/BpfWriter.hpp>
#include <las/LasWriter.hpp>
#include <ply/PlyWriter.hpp>
#include <sbet/SbetWriter.hpp>
#include <derivative/DerivativeWriter.hpp>
#include <text/TextWriter.hpp>
#include <null/NullWriter.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream>
#include <string>
#include <stdio.h> // for funcptr

namespace pdal
{

std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    // filename may actually be a greyhound uri + pipelineId
    std::string http = filename.substr(0, 4);
    if (boost::iequals(http, "http"))
        return "readers.greyhound";

    std::string ext = boost::filesystem::extension(filename);
    std::map<std::string, std::string> drivers;
    drivers["bin"] = "readers.terrasolid";
    drivers["bpf"] = "readers.bpf";
    drivers["cpd"] = "readers.optech";
    drivers["greyhound"] = "readers.greyhound";
    drivers["icebridge"] = "readers.icebridge";
    drivers["las"] = "readers.las";
    drivers["laz"] = "readers.las";
    drivers["nitf"] = "readers.nitf";
    drivers["nsf"] = "readers.nitf";
    drivers["ntf"] = "readers.nitf";
    drivers["pcd"] = "readers.pcd";
    drivers["ply"] = "readers.ply";
    drivers["qi"] = "readers.qfit";
    drivers["rxp"] = "readers.rxp";
    drivers["sbet"] = "readers.sbet";
    drivers["sqlite"] = "readers.sqlite";

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
    drivers["bpf"] = "writers.bpf";
    drivers["csv"] = "writers.text";
    drivers["json"] = "writers.text";
    drivers["las"] = "writers.las";
    drivers["laz"] = "writers.las";
    drivers["mat"] = "writers.matlab";
    drivers["ntf"] = "writers.nitf";
    drivers["pcd"] = "writers.pcd";
    drivers["pclviz"] = "writers.pclvisualizer";
    drivers["ply"] = "writers.ply";
    drivers["sbet"] = "writers.sbet";
    drivers["derivative"] = "writers.derivative";
    drivers["sqlite"] = "writers.sqlite";
    drivers["txt"] = "writers.text";
    drivers["xyz"] = "writers.text";

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


pdal::Options StageFactory::inferWriterOptionsChanges(
    const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);
    boost::to_lower(ext);
    Options options;

    if (boost::algorithm::iequals(ext,".laz"))
    {
        options.add("compression", true);
    }

    PluginManager & pm = PluginManager::getInstance();
    if (boost::algorithm::iequals(ext,".pcd") && pm.createObject("writers.pcd"))
    {
        options.add("format","PCD");
    }

    options.add<std::string>("filename", filename);
    return options;
}

StageFactory::StageFactory(bool no_plugins)
{
    PluginManager & pm = PluginManager::getInstance();
    if (!no_plugins)
    {
        pm.loadAll(PF_PluginType_Filter);
        pm.loadAll(PF_PluginType_Reader);
        pm.loadAll(PF_PluginType_Writer);
    }

    // filters
    PluginManager::initializePlugin(ChipperFilter_InitPlugin);
    PluginManager::initializePlugin(ColorizationFilter_InitPlugin);
    PluginManager::initializePlugin(CropFilter_InitPlugin);
    PluginManager::initializePlugin(DecimationFilter_InitPlugin);
    PluginManager::initializePlugin(FerryFilter_InitPlugin);
    PluginManager::initializePlugin(MergeFilter_InitPlugin);
    PluginManager::initializePlugin(MortonOrderFilter_InitPlugin);
    PluginManager::initializePlugin(RangeFilter_InitPlugin);
    PluginManager::initializePlugin(ReprojectionFilter_InitPlugin);
    PluginManager::initializePlugin(SortFilter_InitPlugin);
    PluginManager::initializePlugin(SplitterFilter_InitPlugin);
    PluginManager::initializePlugin(StatsFilter_InitPlugin);
    PluginManager::initializePlugin(TransformationFilter_InitPlugin);

    // readers
    PluginManager::initializePlugin(BpfReader_InitPlugin);
    PluginManager::initializePlugin(FauxReader_InitPlugin);
    PluginManager::initializePlugin(LasReader_InitPlugin);
    PluginManager::initializePlugin(OptechReader_InitPlugin);
    PluginManager::initializePlugin(PlyReader_InitPlugin);
    PluginManager::initializePlugin(QfitReader_InitPlugin);
    PluginManager::initializePlugin(SbetReader_InitPlugin);
    PluginManager::initializePlugin(TerrasolidReader_InitPlugin);

    // writers
    PluginManager::initializePlugin(BpfWriter_InitPlugin);
    PluginManager::initializePlugin(LasWriter_InitPlugin);
    PluginManager::initializePlugin(PlyWriter_InitPlugin);
    PluginManager::initializePlugin(SbetWriter_InitPlugin);
    PluginManager::initializePlugin(DerivativeWriter_InitPlugin);
    PluginManager::initializePlugin(TextWriter_InitPlugin);
    PluginManager::initializePlugin(NullWriter_InitPlugin);
}

/// Create a stage and return a pointer to the created stage.  Caller takes
/// ownership unless the ownStage argument is true.
///
/// \param[in] stage_name  Type of stage to by created.
/// \param[in] ownStage    Whether the factory should own the stage.
/// \return  Pointer to created stage.
///
Stage *StageFactory::createStage(std::string const& stage_name,
    bool ownStage)
{
    PluginManager& pm = PluginManager::getInstance();
    Stage *s = static_cast<Stage*>(pm.createObject(stage_name));
    if (s && ownStage)
        m_ownedStages.push_back(std::unique_ptr<Stage>(s));
    return s;
}


StringList StageFactory::getStageNames() const
{
    PluginManager & pm = PluginManager::getInstance();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    StringList nv;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Filter ||
            r.second.pluginType == PF_PluginType_Reader ||
            r.second.pluginType == PF_PluginType_Writer)
            nv.push_back(r.first);
    }
    return nv;
}


std::map<std::string, std::string> StageFactory::getStageMap() const
{
    PluginManager& pm = PluginManager::getInstance();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    std::map<std::string, std::string> sm;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Filter ||
            r.second.pluginType == PF_PluginType_Reader ||
            r.second.pluginType == PF_PluginType_Writer)
            sm.insert(std::make_pair(r.first, r.second.description));
    }
    return sm;
}

} // namespace pdal

