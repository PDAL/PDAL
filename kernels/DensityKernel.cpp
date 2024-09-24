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

#include "../filters/HexBinFilter.hpp"
#include "private/density/OGR.hpp"

#include <pdal/util/FileUtils.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "kernels.density",
    "Density Kernel [DEPRECATED]",
    "http://pdal.io/apps/density.html"
};

CREATE_STATIC_KERNEL(DensityKernel, s_info)

std::string DensityKernel::getName() const { return s_info.name; }

void DensityKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "input point cloud file name", m_inputFile).
        setPositional();
    args.add("output,o", "output vector data source", m_outputFile).
        setPositional();
    args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
        "ESRI Shapefile");
    args.add("lyr_name", "OGR layer name to use", m_layerName, "");
    args.add("sample_size", "Sample size for auto-edge length calculation",
        m_sampleSize, 5000U);
    args.add("threshold", "Required cell density", m_density, 15);
    args.add("edge_length", "Length of hex edge", m_edgeLength);
    args.add("hole_cull_area_tolerance", "Tolerance area to "
            "apply to holes before cull", m_cullArea);
    args.add("smooth", "Smooth boundary output", m_doSmooth, true);
    args.add("h3_grid", "Create a grid using H3 (https://h3geo.org/docs) Hexagons",
        m_isH3, false);
    args.add("h3_resolution", "H3 grid resolution: 0 (coarsest) - 15 (finest). See "
        "https://h3geo.org/docs/core-library/restable", m_h3Res, -1);
}


void DensityKernel::outputDensity(pdal::SpatialReference const& reference)
{
    HexBin* hexbin = dynamic_cast<HexBin*>(m_hexbinStage);
    if (!hexbin)
        throw pdal::pdal_error("unable to fetch filters.hexbin stage!");

    hexer::BaseGrid* grid = hexbin->grid();

    OGR writer(m_outputFile, reference.getWKT(), m_isH3, m_driverName, m_layerName);
    writer.writeDensity(*grid);
}


int DensityKernel::execute()
{
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
    Options options;
    options.add("sample_size", m_sampleSize);
    options.add("threshold", m_density);
    options.add("edge_length", m_edgeLength);
    options.add("hole_cull_area_tolerance", m_cullArea);
    options.add("smooth", m_doSmooth);
    options.add("h3_grid", m_isH3);
    options.add("h3_resolution", m_h3Res);
    m_hexbinStage = &(m_manager.makeFilter("filters.hexbin",
        *m_manager.getStage(), options));
    m_manager.execute();
    outputDensity(m_manager.pointTable().anySpatialReference());
    return 0;
}

} // namespace pdal
