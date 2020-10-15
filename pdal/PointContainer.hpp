/******************************************************************************
* Copyright (c) 2015, Hobu Inc.
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

#include <pdal/pdal_types.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/PointLayout.hpp>

namespace pdal
{

class PDAL_DLL PointContainer
{
    friend class PointView;
    friend class PointRef;
private:
    virtual void setFieldInternal(Dimension::Id dim, PointId idx,
        const void *val) = 0;
    virtual void getFieldInternal(Dimension::Id dim, PointId idx,
        void *val) const = 0;
    virtual void swapItems(PointId id1, PointId id2)
        { throw pdal_error("Can't swap items in this container."); }
    virtual void setItem(PointId dst, PointId src)
        { throw pdal_error("Can't set item in this container."); }
    virtual bool compare(Dimension::Id dim, PointId id1, PointId id2) const
        { throw pdal_error("Can't compare items in this container."); }
    virtual PointId getTemp(PointId id)
        { return id; }
    virtual void freeTemp(PointId id)
        {}
public:
    virtual PointLayoutPtr layout() const = 0;
};

} // namespace pdal
