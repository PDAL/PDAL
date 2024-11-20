/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hou.co)
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
*       the documentation and/or key materials provided
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

#include <pdal/Writer.hpp>

namespace pdal
{
namespace copcwriter
{
    struct BaseInfo;
}

class PDAL_EXPORT CopcWriter : public Writer
{
public:
    CopcWriter();
    virtual ~CopcWriter();
    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize(PointTableRef table) override;
    virtual void prepared(PointTableRef table) override;
    virtual void ready(PointTableRef table) override;
    virtual void write(const PointViewPtr view) override;
    virtual void done(PointTableRef table) override;

    void fillForwardList();
    template <typename T>
    void handleHeaderForward(const std::string& s, T& headerVal, const MetadataNode& base);
    void handleHeaderForwards(MetadataNode& forward);
    void handleForwardVlrs(MetadataNode& forward);
    void handleUserVlrs(MetadataNode m);
    void handlePipelineVlr();

    std::unique_ptr<copcwriter::BaseInfo> b;
    bool isRemote;
    std::string remoteFilename;
};

} // namespace pdal

