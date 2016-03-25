/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include "DensityKernel.hpp"
#include "../filters/HexBin.hpp"
#include "OGR.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/plugin.hpp>
#include <pdal/GlobalEnvironment.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.density",
    "Density Kernel",
    "http://pdal.io/kernels/kernels.density.html" );

CREATE_SHARED_PLUGIN(1, 0, DensityKernel, Kernel, s_info)

std::string DensityKernel::getName() const { return s_info.name; }


void DensityKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "input point cloud file name", m_inputFile);
    args.add("output,o", "output vector data source", m_outputFile);
    args.add("lyr_name", "OGR layer name to write into datasource",
        m_layerName);
    args.add("driver,f", "OGR driver name to use ", m_driverName,
        "ESRI Shapefile");
}


void DensityKernel::makePipeline(const std::string& filename)
{
    if (!pdal::FileUtils::fileExists(filename))
        throw pdal_error("File not found: " + filename);

    m_manager = PipelineManagerPtr(new PipelineManager);

    if (filename == "STDIN")
    {
        m_manager->readPipeline(std::cin);
    }
    else if (FileUtils::extension(filename) == ".xml" ||
        FileUtils::extension(filename) == ".json")
    {
        m_manager->readPipeline(filename);
    }
    else
    {
        StageFactory factory;
        std::string driver = factory.inferReaderDriver(filename);

        if (driver.empty())
            throw pdal_error("Cannot determine input file type of " + filename);
        Stage& reader = m_manager->addReader(driver);
        Options ro;
        ro.add("filename", filename);
        reader.setOptions(ro);
    }
    Stage *stage = m_manager->getStage();
    m_hexbinStage = &(m_manager->addFilter("filters.hexbin"));
    if (!m_hexbinStage)
        throw pdal_error("Unable to initialize filters.hexbin!");
    m_hexbinStage->setInput(*stage);
}


void DensityKernel::outputDensity(pdal::SpatialReference const& reference)
{
    HexBin* hexbin = static_cast<pdal::HexBin*>(m_hexbinStage);
    if (!hexbin)
        throw pdal::pdal_error("unable to fetch filters.hexbin stage!");

    hexer::HexGrid* grid = hexbin->grid();

    hexdensity::writer::OGR writer(m_outputFile, reference.getWKT(),
        m_driverName);
    writer.writeDensity(grid);
//     writer.writeBoundary(grid);
}


int DensityKernel::execute()
{
    //ABELL - Is this necessary here - isn't it done with the stages?
    GlobalEnvironment::get().wakeGDALDrivers();
    std::string filename = m_usestdin ? std::string("STDIN") : m_inputFile;
    makePipeline(filename);
    applyExtraStageOptionsRecursive(m_manager->getStage());
    point_count_t output = m_manager->execute();
    PointTableRef table = m_manager->pointTable();
    outputDensity(table.anySpatialReference());
    return 0;
}

} // namespace pdal
