/******************************************************************************
* Copyright (c) 2015, Brad Chambers (brad.chambers@gmail.com)
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

#include "HeightAboveGroundKernel.hpp"

#include "PCLConversions.hpp"

#include <boost/program_options.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pdal/BufferReader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Writer.hpp>

#include <memory>
#include <string>
#include <vector>

namespace po = boost::program_options;

CREATE_KERNEL_PLUGIN(height, pdal::HeightAboveGroundKernel)

namespace pdal
{

void HeightAboveGroundKernel::validateSwitches()
{
    if (m_input_file == "")
        throw app_usage_error("--input/-i required");
    if (m_ground_file == "" && !m_use_classification)
        throw app_usage_error("--ground/-g or --use_classification/-c required");
    if (m_output_file == "")
        throw app_usage_error("--output/-o required");

    // should probably verify that the output is BPF
    // should probably also verify that there is a classification dimension if --use_classification == true
}

void HeightAboveGroundKernel::addSwitches()
{
    po::options_description* options = new po::options_description("file options");
    options->add_options()
    ("input,i", po::value<std::string>(&m_input_file)->default_value(""), "input file name")
    ("ground,g", po::value<std::string>(&m_ground_file)->default_value(""), "ground file name")
    ("output,o", po::value<std::string>(&m_output_file)->default_value(""), "output file name")
    ("use_classification,c", po::bool_switch(&m_use_classification)->default_value(false), "use existing classification labels?")
    ;

    addSwitchSet(options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("ground", 1);
    addPositionalSwitch("output", 1);
}

int HeightAboveGroundKernel::execute()
{
    // we require separate contexts for the input and ground files
    PointContextRef input_ctx;
    PointContextRef ground_ctx;

    // because we are appending HeightAboveGround to the input buffer, we must
    // register it's Dimension
    input_ctx.registerDim(Dimension::Id::HeightAboveGround);

    // StageFactory will be used to create required stages
    StageFactory f;

    // setup the reader, inferring driver type from the filename
    std::string reader_driver = f.inferReaderDriver(m_input_file);
    std::unique_ptr<Reader> input(f.createReader(reader_driver));
    Options readerOptions;
    readerOptions.add("filename", m_input_file);
    input->setOptions(readerOptions);

    // go ahead and execute to get the PointBuffer
    input->prepare(input_ctx);
    PointBufferSet pbSetInput = input->execute(input_ctx);
    PointBufferPtr input_buf = *pbSetInput.begin();

    PointBufferSet pbSetGround;
    PointBufferPtr ground_buf;

    if (m_use_classification)
    {
        // the user has indicated that the classification dimension exists, so
        // we will find all ground returns
        Option source("source",
                      "import numpy as np\n"
                      "def yow1(ins,outs):\n"
                      "  cls = ins['Classification']\n"
                      "  keep_classes = [2]\n"
                      "  keep = np.equal(cls, keep_classes[0])\n"
                      "  outs['Mask'] = keep\n"
                      "  return True\n"
                     );
        Option module("module", "MyModule");
        Option function("function", "yow1");
        Options opts;
        opts.add(source);
        opts.add(module);
        opts.add(function);

        // and create a PointBuffer of only ground returns
        std::unique_ptr<Filter> pred(f.createFilter("filters.predicate"));
        pred->setOptions(opts);
        pred->setInput(input.get());
        pred->prepare(ground_ctx);
        pbSetGround = pred->execute(ground_ctx);
        ground_buf = *pbSetGround.begin();
    }
    else
    {
        // the user has provided a file containing only ground returns, setup
        // the reader, inferring driver type from the filename
        std::string ground_driver = f.inferReaderDriver(m_ground_file);
        std::unique_ptr<Reader> ground(f.createReader(ground_driver));
        Options ro;
        ro.add("filename", m_ground_file);
        ground->setOptions(ro);

        // go ahead and execute to get the PointBuffer
        ground->prepare(ground_ctx);
        pbSetGround = ground->execute(ground_ctx);
        ground_buf = *pbSetGround.begin();
    }

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;

    // convert the input PointBuffer to a PointCloud
    CloudPtr cloud(new Cloud);
    BOX3D const& bounds = input_buf->calculateBounds();
    pclsupport::PDALtoPCD(*input_buf, *cloud, bounds);

    // convert the ground PointBuffer to a PointCloud
    CloudPtr cloud_g(new Cloud);
    // here, we offset the ground cloud by the input bounds so that the two are aligned
    pclsupport::PDALtoPCD(*ground_buf, *cloud_g, bounds);

    // create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // create the filtering object and project ground returns into xy plane
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_g);
    proj.setModelCoefficients(coefficients);
    CloudPtr cloud_projected(new Cloud);
    proj.filter(*cloud_projected);

    // setup the KdTree
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(cloud_projected);

    // loop over all points in the input cloud, finding the nearest neighbor in
    // the ground returns (XY plane only), and calculating the difference in z
    int32_t k = 1;
    for (size_t idx = 0; idx < cloud->points.size(); ++idx)
    {
        // Search for nearesrt neighbor of the query point
        std::vector<int32_t> neighbors(k);
        std::vector<float> distances(k);
        PointT temp_pt = cloud->points[idx];
        temp_pt.z = 0.0f;
        int num_neighbors = tree.nearestKSearch(temp_pt, k, neighbors, distances);

        double hag = cloud->points[idx].z - cloud_g->points[neighbors[0]].z;
        input_buf->setField(Dimension::Id::HeightAboveGround, idx, hag);
    }

    // populate BufferReader with the input PointBuffer, which now has the
    // HeightAboveGround dimension
    BufferReader bufferReader;
    bufferReader.addBuffer(input_buf);

    // we require that the output be BPF for now, to house our non-standard
    // dimension
    Options wo;
    wo.add("filename", m_output_file);
    std::unique_ptr<Writer> writer(f.createWriter("writers.bpf"));
    writer->setOptions(wo);
    writer->setInput(&bufferReader);
    writer->prepare(input_ctx);
    writer->execute(input_ctx);

    return 0;
}

} // namespace pdal
