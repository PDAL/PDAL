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

#include <pdal/pdal_types.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace e57
{
class ImageFile;
class Scan;
}

namespace pdal
{

class PDAL_DLL E57Reader: public Reader, public Streamable
{
    class PDAL_LOCAL ChunkReader;

public:
    E57Reader();
    E57Reader(std::string filename);
    ~E57Reader();
    E57Reader(const E57Reader &) = delete;
    E57Reader& operator=(const E57Reader&) = delete;

    std::string getName() const override;

    /// Get the total number of points in the scan
    point_count_t getNumberPoints() const;

    /// Gets the scan index of a given point index
    int getScanIndex(point_count_t) const;

    /// Gets the dimensions present within the set of clouds
    std::set<std::string> getDimensions();

    /// returns the scans
    std::vector<std::shared_ptr<e57::Scan>> getScans() const;

private:

    /* Pdal section */
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void initialize() override;
    virtual bool processOne(PointRef& point) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual QuickInfo inspect() override;

    void openFile(const std::string &filename);
    void closeFile();
    void setupReader(point_count_t pointNumber);
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

} // namespace pdal
