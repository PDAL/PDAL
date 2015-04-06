/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "pdal/Dimension.hpp"

namespace pdal
{

class PDAL_DLL PointLayout
{
public:
    PointLayout();
    virtual ~PointLayout() {}

    void finalize();

    void registerDims(std::vector<Dimension::Id::Enum> ids);
    void registerDims(Dimension::Id::Enum *id);
    void registerDim(Dimension::Id::Enum id);

    // Register the dimension with the requested type if it hasn't already
    // been registered with a "larger" type.  If the type of the dimension
    // is already larger, this does nothing.
    void registerDim(Dimension::Id::Enum id, Dimension::Type::Enum type);

    // The type and size are REQUESTS, not absolutes.  If someone else
    // has already registered with the same name, you get the existing
    // dimension size/type.
    Dimension::Id::Enum assignDim(
            const std::string& name,
            Dimension::Type::Enum type);

    Dimension::Id::Enum registerOrAssignDim(
            const std::string name,
            Dimension::Type::Enum type);

    DimTypeList dimTypes() const;
    DimType findDimType(const std::string& name) const;
    Dimension::Id::Enum findDim(const std::string& name) const;
    Dimension::Id::Enum findProprietaryDim(const std::string& name) const;
    std::string dimName(Dimension::Id::Enum id) const;

    // @return whether or not the PointLayout contains a given id
    bool hasDim(Dimension::Id::Enum id) const;

    // @return reference to vector of currently used dimensions
    const Dimension::IdList& dims() const;

    // @return the current type for a given id
    Dimension::Type::Enum dimType(Dimension::Id::Enum id) const;

    // @return the current size in bytes of the dimension
    //         with the given id.
    size_t dimSize(Dimension::Id::Enum id) const;
    size_t dimOffset(Dimension::Id::Enum id) const;
    size_t pointSize() const;

    const Dimension::Detail *dimDetail(Dimension::Id::Enum id) const;

private:
    virtual bool update(Dimension::Detail dd, const std::string& name);

    Dimension::Type::Enum resolveType(
            Dimension::Type::Enum t1,
            Dimension::Type::Enum t2);

protected:
    std::vector<Dimension::Detail> m_detail;
    Dimension::IdList m_used;
    std::map<std::string, Dimension::Id::Enum> m_propIds;
    int m_nextFree;
    std::size_t m_pointSize;
    bool m_finalized;
};

typedef PointLayout* PointLayoutPtr;

} // namespace pdal

