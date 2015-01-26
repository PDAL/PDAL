/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#pragma once

#include <pdal/Kernel.hpp>

namespace pdal
{

class PDAL_DLL TranslateKernel : public Kernel
{
public:
    SET_KERNEL_NAME ("translate", "Translate Kernel")
    SET_KERNEL_LINK ("http://pdal.io/kernels/kernels.translate.html")
 
    TranslateKernel();
    int execute();

private:
    void addSwitches();
    void validateSwitches();

    std::unique_ptr<Stage> makeReader(Options readerOptions);
    Stage* makeTranslate(Options translateOptions, Stage* reader);
    void forwardMetadata(Options & options, Metadata metadata);

    std::string m_inputFile;
    std::string m_outputFile;
    bool m_bCompress;
    pdal::SpatialReference m_input_srs;
    pdal::SpatialReference m_output_srs;
    BOX3D m_bounds;
    std::string m_wkt;
    bool m_bForwardMetadata;
    uint32_t m_decimation_step;
    uint32_t m_decimation_offset;
    double m_decimation_leaf_size;
    std::string m_decimation_method;
    point_count_t m_decimation_limit;

};

} // namespace pdal
