/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc.
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
 *     * Neither the name of Helix Re Inc. nor the
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

#include <E57Format.h>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include "Scan.hpp"
#include "Utils.hpp"

namespace pdal
{
class PDAL_DLL E57Reader : public Reader, public Streamable
{
public:
    E57Reader();
    std::string getName() const;

private:
    /* Pdal section */
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void initialize() override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;
    virtual bool processOne(PointRef&) override;
    virtual void ready(PointTableRef&) override;
    virtual QuickInfo inspect() override;
    virtual void addArgs(ProgramArgs& args) override;

    // Private members
    bool fillPoint(PointRef& point);
    point_count_t readNextBatch();
    void setupReader();
    void initializeBuffers();

    std::unique_ptr<e57::ImageFile> m_imf;
    std::unique_ptr<e57::VectorNode> m_data3D;
    std::unique_ptr<e57::CompressedVectorReader> m_reader;
    std::unique_ptr<e57::StructureNode> m_e57PointPrototype;
    std::shared_ptr<e57::Scan> m_scan;

    std::map<std::string, std::vector<double>> m_doubleBuffers;
    std::vector<e57::SourceDestBuffer> m_destBuffers;

    point_count_t m_currentIndex;
    point_count_t m_pointsInCurrentBatch;
    point_count_t m_defaultChunkSize;
    signed int m_currentScan;

    pdal::StringList m_extraDimsSpec;
    std::unique_ptr<e57plugin::ExtraDims> m_extraDims;
};

} // namespace pdal