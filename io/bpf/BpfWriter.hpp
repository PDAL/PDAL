/******************************************************************************
* Copyright (c) 2015, Hobu Inc., hobu@hobu.co
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

// BPF is an NGA specification for point cloud data. The specification can be
// found at https://nsgreg.nga.mil/doc/view?i=4202

#pragma once

#include "BpfHeader.hpp"

#include <pdal/pdal_export.hpp>
#include <pdal/FlexWriter.hpp>
#include <pdal/util/OStream.hpp>

#include <vector>

extern "C" int32_t BpfWriter_ExitFunc();
extern "C" PF_ExitFunc BpfWriter_InitPlugin();

namespace pdal
{

class PDAL_DLL BpfWriter : public FlexWriter
{
public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    OLeStream m_stream;
    BpfHeader m_header;
    BpfDimensionList m_dims;
    std::vector<uint8_t> m_extraData;
    std::vector<BpfUlemFile> m_bundledFiles;

    virtual void processOptions(const Options& options);
    virtual void readyTable(PointTableRef table);
    virtual void readyFile(const std::string& filename);
    virtual void writeView(const PointViewPtr data);
    virtual void doneFile();

    double getAdjustedValue(const PointView* data, BpfDimension& bpfDim,
        PointId idx);
    void loadBpfDimensions(PointLayoutPtr layout);
    void writePointMajor(const PointView* data);
    void writeDimMajor(const PointView* data);
    void writeByteMajor(const PointView* data);
    void writeCompressedBlock(char *buf, size_t size);
};

} // namespace pdal
