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

#include <string>

#include <pdal/pdal_types.hpp>
#include <E57Format.h>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include "Scan.hpp"

namespace pdal {
class E57Reader: public Reader, public Streamable
{
    class ChunkReader {
    public:
        ChunkReader(const pdal::point_count_t &pointOffset,
            const pdal::point_count_t &maxPointRead,
            const std::shared_ptr<e57::Scan> &scan,
            const std::set<std::string> &e57Dimensions);

        ~ChunkReader();

        // returns false if the index falls out of the [pointOffset,pointOffset + m_maxPointRead] interval
        bool isInScope(pdal::point_count_t index) const;

        bool isInChunk(pdal::point_count_t index) const;

        void setPoint(pdal::point_count_t pointIndex, pdal::PointRef point,
                      const std::set<std::string> &e57Dimensions) const;

        // Reads a new chunk of data
        pdal::point_count_t read(pdal::point_count_t index);

    private:
        pdal::point_count_t m_startIndex;
        pdal::point_count_t m_pointOffset;
        pdal::point_count_t m_maxPointRead;
        const pdal::point_count_t m_defaultChunkSize;
        std::map<std::string, std::vector<double>> m_doubleBuffers;
        std::vector<e57::SourceDestBuffer> m_e57buffers;
        std::unique_ptr<e57::CompressedVectorReader> m_dataReader;
        std::shared_ptr<e57::Scan> m_scan;
    };

public:
    E57Reader()
    {}

    E57Reader(std::string filename);
    ~E57Reader();
    E57Reader(const E57Reader &) = delete;
    E57Reader& operator=(const E57Reader&) = delete;

    std::string getName() const;

    /// Get the total number of points in the scan
    point_count_t getNumberPoints() const;

    /// Gets the scan index of a given point index
    int getScanIndex(pdal::point_count_t) const;

    /// Gets the dimensions present within the set of clouds
    std::set<std::string> getDimensions();

    /// returns the scans
   std::vector<std::shared_ptr<e57::Scan>> getScans() const;

private:

    /* Pdal section */
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void initialize();
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual QuickInfo inspect();

    void openFile(const std::string &filename);
    void closeFile();
    void setupReader(pdal::point_count_t pointNumber);
    point_count_t extractNumberPoints() const;
    void extractScans();

    // members
    std::unique_ptr<e57::ImageFile> m_imf; 
    std::vector<std::shared_ptr<e57::Scan>> m_scans;

    // Allows construction by filename
    std::string m_filenameManual;

    // E57 dimensions that are common to all scans
    std::set<std::string> m_validDimensions;

    // Chunk data reader
    std::unique_ptr<ChunkReader> m_chunk;
    point_count_t m_currentPoint; // Variable for streaming mode

    // Cache the total number of points
    point_count_t m_pointCount;
};
}
