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

#include <pdal/Utils.hpp>
#include <pdal/PluginManager.hpp>

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

#include <pdal/BufferReader.hpp>
#include <faux/FauxReader.hpp>
#include <las/LasReader.hpp>
#include <las/LasWriter.hpp>
#include <bpf/BpfReader.hpp>
#include <bpf/BpfWriter.hpp>
#include <sbet/SbetReader.hpp>
#include <sbet/SbetWriter.hpp>
#include <qfit/QfitReader.hpp>
#include <terrasolid/TerrasolidReader.hpp>
#include <text/TextWriter.hpp>

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
    PluginManager & pm = PluginManager::getInstance();

    // filename may actually be a greyhound uri + pipelineId
    std::string http = filename.substr(0, 4);
    if (boost::iequals(http, "http"))
        return "readers.greyhound";

    std::string ext = boost::filesystem::extension(filename);
    std::map<std::string, std::string> drivers;
    drivers["las"] = "readers.las";
    drivers["laz"] = "readers.las";
    drivers["bin"] = "readers.terrasolid";
    drivers["greyhound"] = "readers.greyhound";
    drivers["qi"] = "readers.qfit";
    drivers["nitf"] = "readers.nitf";
    drivers["ntf"] = "readers.nitf";
    drivers["nsf"] = "readers.nitf";
    drivers["bpf"] = "readers.bpf";
    drivers["sbet"] = "readers.sbet";
    drivers["icebridge"] = "readers.icebridge";
    drivers["sqlite"] = "readers.sqlite";
    drivers["rxp"] = "readers.rxp";
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
    PluginManager & pm = PluginManager::getInstance();

    std::string ext = boost::filesystem::extension(filename);

    boost::to_lower(ext);

    std::map<std::string, std::string> drivers;
    drivers["bpf"] = "writers.bpf";
    drivers["las"] = "writers.las";
    drivers["laz"] = "writers.las";
    drivers["pcd"] = "writers.pcd";
    drivers["pclviz"] = "writers.pclvisualizer";
    drivers["sbet"] = "writers.sbet";
    drivers["csv"] = "writers.text";
    drivers["json"] = "writers.text";
    drivers["xyz"] = "writers.text";
    drivers["txt"] = "writers.text";
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
    PluginManager::initializePlugin(FauxReader_InitPlugin);
    PluginManager::initializePlugin(LasReader_InitPlugin);
    PluginManager::initializePlugin(BpfReader_InitPlugin);
    PluginManager::initializePlugin(QfitReader_InitPlugin);
    PluginManager::initializePlugin(SbetReader_InitPlugin);
    PluginManager::initializePlugin(TerrasolidReader_InitPlugin);
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
    PluginManager::initializePlugin(BpfWriter_InitPlugin);
    PluginManager::initializePlugin(LasWriter_InitPlugin);
    PluginManager::initializePlugin(SbetWriter_InitPlugin);
    PluginManager::initializePlugin(TextWriter_InitPlugin);
}

/// Create a stage and return a pointer to the created stage.  Caller takes
/// ownership and is responsible for stage cleanup.
///
/// \param[in] stage_name  Type of stage to by created.
/// \return  Pointer to created stage.
///
Stage *StageFactory::createStage(std::string const& stage_name)
{
    PluginManager& pm = PluginManager::getInstance();

    Stage *stage = (Stage *)pm.createObject(stage_name);
    if (!stage)
        if (pm.guessLoadByPath(stage_name) == 0)
            stage = (Stage *)pm.createObject(stage_name);
    return stage;
}


StringList StageFactory::getStageNames()
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


std::map<std::string, std::string> StageFactory::getStageMap()
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

