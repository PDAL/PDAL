/******************************************************************************
* Copyright (c) 2020, Ryan Pals, ryan@hobu.co
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

#include <pdal/pdal_features.hpp>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <nlohmann/json.hpp>

#include <vector>

namespace pdal
{

namespace hdf5
{
    class Handler;
    class DimInfo;
}

class PDAL_DLL HdfReader : public pdal::Reader, public pdal::Streamable
{
public:
    HdfReader();
    std::string getName() const override;

private:
    std::unique_ptr<hdf5::Handler> m_hdf5Handler;
    point_count_t m_index;

    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual void ready(PointTableRef table) override;
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual bool processOne(PointRef& point) override;
    virtual void done(PointTableRef table) override;

    NL::json m_pathDimJson;
    std::map<std::string,std::string> m_pathDimMap;
    Dimension::IdList m_idlist;
    void parseDimensions();

    HdfReader& operator=(const HdfReader&);   // Not implemented.
    HdfReader(const HdfReader&);              // Not implemented.
};

} // namespace pdal
