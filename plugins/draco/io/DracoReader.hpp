/******************************************************************************
* Copyright (c) 2020 Hobu, Inc. (info@hobu.co)
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

#include <iostream>

#include <pdal/Reader.hpp>
#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/decode.h>

namespace pdal
{

class PDAL_DLL DracoReader : public Reader
{
public:

    DracoReader() = default;
    std::string getName() const;
private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    void addOneDimension(Dimension::Id id, const draco::PointAttribute* attr, PointLayoutPtr layout, int index, int attNum);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void prepared(PointTableRef);
    virtual void ready(PointTableRef);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);

    struct DimensionInfo {
        Dimension::Id pdalId;
        const draco::PointAttribute *attr;
        Dimension::Type pdalType;
        int attNum;//eg POSITION = [ X, Y, Z ], Y's attNum would be 1
    };
    std::vector<DimensionInfo> m_dimensions;
    DracoReader(const DracoReader&) = delete;
    DracoReader& operator=(const DracoReader&) = delete;

    std::istream* m_istreamPtr;
    std::vector<char> m_data;
    draco::DecoderBuffer m_draco_buffer;
    std::unique_ptr<draco::PointCloud> m_pc;

};

} // namespace pdal
