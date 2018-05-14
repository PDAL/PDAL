/******************************************************************************
* Copyright (c) 2018, RIEGL Laser Measurement Systems GmbH (support@riegl.com)
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
*     * Neither the name of Hobu, Inc., Flaxen Geo Consulting or RIEGL
*       Laser Measurement Systems GmbH nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <memory>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include "RdbPointcloud.hpp"

namespace pdal
{

//______________________________________________________________________________
/*!
 * PDAL reader for RIEGL RDB point cloud files
 */
class PDAL_DLL RdbReader : public pdal::Reader, public pdal::Streamable
{
public:
    RdbReader();
    virtual ~RdbReader() override;
    virtual std::string getName() const override;

private:
    virtual QuickInfo inspect() override;
    virtual void addArgs(ProgramArgs& args) override;

    virtual void initialize() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual bool processOne(PointRef& point) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual void done(PointTableRef table) override;

    static void readMetadata(RdbPointcloud &reader, MetadataNode root);
    static std::string getSpatialReferenceSystem(const RdbPointcloud &reader);

private:
    std::unique_ptr<RdbPointcloud> m_pointcloud;
    std::string                    m_filter;
    bool                           m_extras;
};

} // namespace pdal
