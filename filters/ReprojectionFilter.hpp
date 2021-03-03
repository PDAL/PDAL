/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

#include <memory>

namespace pdal
{

class SrsTransform;

class PDAL_DLL ReprojectionFilter : public Filter, public Streamable
{
public:
    ReprojectionFilter();
    ~ReprojectionFilter();

    std::string getName() const;

private:
    ReprojectionFilter& operator=(const ReprojectionFilter&) = delete;
    ReprojectionFilter(const ReprojectionFilter&) = delete;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual PointViewSet run(PointViewPtr view);
    virtual bool processOne(PointRef& point);
    virtual void spatialReferenceChanged(const SpatialReference& srs);
    virtual void prepared(PointTableRef table);

    void createTransform(const SpatialReference& srs);

    SpatialReference m_inSRS;
    SpatialReference m_outSRS;
    bool m_inferInputSRS;
    std::unique_ptr<SrsTransform> m_transform;
    std::vector<std::string> m_inAxisOrderingArg;
    std::vector<std::string> m_outAxisOrderingArg;
    std::vector<int> m_inAxisOrdering;
    std::vector<int> m_outAxisOrdering;
    bool m_errorOnFailure;
};

} // namespace pdal
