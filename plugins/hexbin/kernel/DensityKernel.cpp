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

#include <pdal/GDALUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/plugin.hpp>

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
    args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
        "ESRI Shapefile");
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
    gdal::registerDrivers();

    if (m_inputFile == "STDIN" ||
        (FileUtils::extension(m_inputFile) == ".xml" ||
        FileUtils::extension(m_inputFile) == ".json"))
    {
        m_manager.readPipeline(m_inputFile);
    }
    else
    {
        m_manager.makeReader(m_inputFile, "");
    }
    m_hexbinStage = &(m_manager.makeFilter("filters.hexbin",
        *m_manager.getStage()));
    m_manager.execute();
    outputDensity(m_manager.pointTable().anySpatialReference());
    return 0;
}

} // namespace pdal
