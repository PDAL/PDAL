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

#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.density",
    "Density Kernel",
    "http://pdal.io/kernels/kernels.density.html" );

CREATE_STATIC_PLUGIN(1, 0, DensityKernel, Kernel, s_info)

std::string DensityKernel::getName() const { return s_info.name; }

DensityKernel::DensityKernel()
{}


void DensityKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "input point cloud file name", m_inputFile);
    args.add("output,o", "output vector geometry file name", m_outputFile);
    args.add("lyr_name", "OGR layer name to write into datasource",
        m_layerName);
    args.add("driver,f", "OGR driver name to use ", m_driverName,
        "ESRI Shapefile");
}

void DensityKernel::setup(const std::string& filename)
{
    m_manager = makePipeline(filename);
    Stage *stage = m_manager->getStage();
    m_hexbinStage = &(m_manager->addFilter("filters.hexbin"));
    if (!m_hexbinStage) {
        throw pdal_error("Unable to compute boundary -- "
            "http://github.com/hobu/hexer is not linked. "
            "See the \"boundary\" member in \"stats\" for a coarse "
            "bounding box");
    }
    m_hexbinStage->setInput(*stage);

}
PipelineManagerPtr DensityKernel::makePipeline(const std::string& filename)
{
    if (!pdal::FileUtils::fileExists(filename))
        throw pdal_error("File not found: " + filename);

    PipelineManagerPtr output(new PipelineManager);

    if (filename == "STDIN")
    {
        output->readPipeline(std::cin);
    }
    else if (FileUtils::extension(filename) == ".xml" ||
        FileUtils::extension(filename) == ".json")
    {
        output->readPipeline(filename);
    }
    else
    {
        StageFactory factory;
        std::string driver = factory.inferReaderDriver(filename);

        if (driver.empty())
            throw pdal_error("Cannot determine input file type of " + filename);
        Stage& reader = output->addReader(driver);
        Options ro;
        ro.add("filename", filename);
        reader.setOptions(ro);
    }
    return output;
}

int DensityKernel::execute()
{

    std::string filename = m_usestdin ? std::string("STDIN") : m_inputFile;
    setup(filename);
    applyExtraStageOptionsRecursive(m_manager->getStage());
    m_manager->execute();
    return 0;
}


}
