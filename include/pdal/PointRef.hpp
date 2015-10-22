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

#include <pdal/PointContainer.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

class PDAL_DLL PointRef
{
public:
    PointRef(PointContainer *container, PointId idx) :
        m_container(container), m_layout(container->layout()), m_idx(idx)
    {}

    template<class T>
    T getFieldAs(Dimension::Id::Enum dim) const
    {
        T val;
        bool success = false;
        Everything e;
        Dimension::Type::Enum type = m_layout->dimDetail(dim)->type();

        m_container->getFieldInternal(dim, m_idx, &e);
        switch (type)
        {
        case Dimension::Type::Unsigned8:
            success = Utils::numericCast(e.u8, val);
            break;
        case Dimension::Type::Unsigned16:
            success = Utils::numericCast(e.u16, val);
            break;
        case Dimension::Type::Unsigned32:
            success = Utils::numericCast(e.u32, val);
            break;
        case Dimension::Type::Unsigned64:
            success = Utils::numericCast(e.u64, val);
            break;
        case Dimension::Type::Signed8:
            success = Utils::numericCast(e.s8, val);
            break;
        case Dimension::Type::Signed16:
            success = Utils::numericCast(e.s16, val);
            break;
        case Dimension::Type::Signed32:
            success = Utils::numericCast(e.s32, val);
            break;
        case Dimension::Type::Signed64:
            success = Utils::numericCast(e.s64, val);
            break;
        case Dimension::Type::Float:
            success = Utils::numericCast(e.f, val);
            break;
        case Dimension::Type::Double:
            success = Utils::numericCast(e.d, val);
            break;
        case Dimension::Type::None:
            break;
        }
        if (!success)
        {
            std::ostringstream oss;
            oss << "Unable to fetch data and convert as requested: ";
            oss << Dimension::name(dim) << ":" <<
                Dimension::interpretationName(type) <<
                "(" << (double)val << ") -> " << Utils::typeidName<T>();
            throw pdal_error(oss.str());
        }
        return val;
    }

    template<typename T>
    void setField(Dimension::Id::Enum dim, T val)
    {
        Dimension::Type::Enum type = m_layout->dimDetail(dim)->type();
        Everything e;
        bool success = false;

        switch (type)
        {
        case Dimension::Type::Unsigned8:
            success = Utils::numericCast(val, e.u8);
            break;
        case Dimension::Type::Unsigned16:
            success = Utils::numericCast(val, e.u16);
            break;
        case Dimension::Type::Unsigned32:
            success = Utils::numericCast(val, e.u32);
            break;
        case Dimension::Type::Unsigned64:
            success = Utils::numericCast(val, e.u64);
            break;
        case Dimension::Type::Signed8:
            success = Utils::numericCast(val, e.s8);
            break;
        case Dimension::Type::Signed16:
            success = Utils::numericCast(val, e.s16);
            break;
        case Dimension::Type::Signed32:
            success = Utils::numericCast(val, e.s32);
            break;
        case Dimension::Type::Signed64:
            success = Utils::numericCast(val, e.s64);
            break;
        case Dimension::Type::Float:
            success = Utils::numericCast(val, e.f);
            break;
        case Dimension::Type::Double:
            success = Utils::numericCast(val, e.d);
            break;
        case Dimension::Type::None:
            break;
        }
        if (success)
            m_container->setFieldInternal(dim, m_idx, &e);
    }

    void setPointId(PointId idx)
        { m_idx = idx; }

private:
    PointContainer *m_container;
    PointLayout *m_layout;
    PointId m_idx;
};

} // namespace pdal
