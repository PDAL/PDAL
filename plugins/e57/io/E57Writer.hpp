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
#include <pdal/Streamable.hpp>
#include <pdal/Writer.hpp>
#include <pdal/pdal_types.hpp>
#include "Utils.hpp"

namespace pdal
{
class PDAL_DLL E57Writer : public pdal::Writer, public pdal::Streamable
{

    class PDAL_DLL ChunkWriter
    {
    public:
        ChunkWriter(const std::vector<std::string>& dimensionsToWrite,
                    e57::CompressedVectorNode& vectorNode);

        void write(pdal::PointRef& point,
                   std::unique_ptr<e57plugin::ExtraDims>& extraDims);

        void finalise();

        inline uint64_t getColorLimit()
        {
            return m_colorLimit - 1;
        }

        inline uint64_t getIntensityLimit()
        {
            if (m_intensityLimit != 1)
            {
                return m_intensityLimit - 1;
            }
            return m_intensityLimit;
        }

    private:
        const pdal::point_count_t m_defaultChunkSize;
        pdal::point_count_t m_currentIndex;
        std::map<std::string, std::vector<double>> m_doubleBuffers;
        std::vector<e57::SourceDestBuffer> m_e57buffers;
        std::unique_ptr<e57::CompressedVectorWriter> m_dataWriter;
        uint64_t m_colorLimit;
        uint64_t m_intensityLimit;
    };

public:
    E57Writer();
    ~E57Writer();
    E57Writer(const E57Writer&) = delete;
    E57Writer& operator=(const E57Writer&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual bool processOne(PointRef& point);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);
    virtual void addDimensions(PointLayoutPtr);

    void setupFileHeader();

    void setupWriter();

    // Writer parameters
    std::string m_filename;
    bool m_doublePrecision;

    // e57 file objects
    std::unique_ptr<e57::ImageFile> m_imageFile;
    std::unique_ptr<e57::StructureNode> m_rootNode;
    std::unique_ptr<ChunkWriter> m_chunkWriter;
    std::unique_ptr<e57::StructureNode> m_scanNode;

    // What do we write?
    std::vector<std::string> m_dimensionsToWrite;
    pdal::StringList m_extraDimsSpec;
    std::unique_ptr<e57plugin::ExtraDims> m_extraDims;

    // Bounds
    BOX3D m_bbox;
};
} // namespace pdal
