/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include "oci_wrapper.h"

#include <cpl_port.h>

#include <pdal/pdal_internal.hpp>
#include <pdal/Schema.hpp>

namespace pdal
{
namespace drivers
{
namespace oci
{

typedef std::shared_ptr<OWConnection> Connection;
typedef std::shared_ptr<OWStatement> Statement;

class connection_failed : public pdal_error
{
public:
    connection_failed(std::string const& msg)
        : pdal_error(msg)
    {}
};


class buffer_too_small : public pdal_error
{
public:
    buffer_too_small(std::string const& msg)
        : pdal_error(msg)
    {}
};


class read_error : public pdal_error
{
public:
    read_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class Block
{
private:
    struct Scale
    {
        friend class Block;

        double m_scale;
        double m_offset;
    };

public:
    Block(Connection connection);
    ~Block();

    point_count_t numRemaining() const
        { return m_num_remaining; }
    void setNumRemaining(point_count_t num_remaining)
        { m_num_remaining = num_remaining; }
    point_count_t numRead() const
        { return num_points - m_num_remaining; }
    point_count_t numPoints() const
        { return num_points; }
    double xOffset() const
        { return m_scaleX.m_offset; }
    double yOffset() const
        { return m_scaleY.m_offset; }
    double zOffset() const
        { return m_scaleZ.m_offset; }
    double xScale() const
        { return m_scaleX.m_scale; }
    double yScale() const
        { return m_scaleY.m_scale; }
    double zScale() const
        { return m_scaleZ.m_scale; }
    char *data() const
        { return (char *)chunk.data(); }
    schema::Orientation orientation() const
        { return m_orientation; }
    void initialize(Schema *s);

    int32_t obj_id;
    int32_t blk_id;
    sdo_geometry *blk_extent;
    sdo_orgscl_type *blk_domain;
    double pcblk_min_res;
    double pcblk_max_res;
    int32_t num_points;
    int32_t num_unsorted_points;
    int32_t pt_sort_dim;
    std::vector<uint8_t> chunk;
    OCILobLocator *locator;
    Connection m_connection;
    sdo_pc* pc;
    int32_t m_num_remaining;
    Scale m_scaleX;
    Scale m_scaleY;
    Scale m_scaleZ;
    schema::Orientation m_orientation;
};
typedef std::shared_ptr<Block> BlockPtr;

Connection connect(std::string connSpec);
Schema fetchSchema(Statement stmt, BlockPtr block);

} // namespace oci
} // namespace drivers
} // namespace pdal

