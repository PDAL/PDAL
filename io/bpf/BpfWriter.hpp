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

#pragma once

#include <vector>

#include <pdal/OStream.hpp>
#include <pdal/Writer.hpp>

#include "BpfHeader.hpp"

namespace pdal
{

class PDAL_DLL BpfWriter : public Writer
{
public:
    SET_STAGE_NAME("writers.bpf",
        "\"Binary Point Format\" (BPF) writer support. "
        "BPF is a simple \n"
        "DoD and research format that is used by some sensor and \n"
        "processing chains.");
    SET_STAGE_LINK("http://pdal.io/stages/writers.bpf.html")

    static Options getDefaultOptions();

private:
    OLeStream m_stream;
    BpfHeader m_header;
    BpfDimensionList m_dims;

    virtual void processOptions(const Options& options);
    virtual void ready(PointContextRef ctx);
    virtual void write(const PointBuffer& buf);
    virtual void done(PointContextRef ctx);

    double getAdjustedValue(const PointBuffer& buf, BpfDimension& bpfDim,
        PointId idx);
    void loadBpfDimensions(PointContextRef ctx);
    void writePointMajor(const PointBuffer& buf);
    void writeDimMajor(const PointBuffer& buf);
    void writeByteMajor(const PointBuffer& buf);
    void writeCompressedBlock(char *buf, size_t size);
};

} // namespace pdal
