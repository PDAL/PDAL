/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#include <string.h>
#include <cctype>
#include <limits>

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>

#include "DracoWriter.hpp"


namespace pdal {

static PluginInfo const s_info
{
    "writers.draco",
    "Write data using Draco.",
    "http://pdal.io/stages/writers.draco.html"
};

struct DracoWriter::Args
{
    std::string m_arrayName;
};


CREATE_SHARED_STAGE(DracoWriter, s_info)

// void writeAttributeValue(DracoWriter::DimBuffer& dim,
//     PointRef& point, size_t idx)
// {
//     Everything e;
//
//     switch (dim.m_type)
//     {
//     case Dimension::Type::Double:
//         e.d = point.getFieldAs<double>(dim.m_id);
//         break;
//     case Dimension::Type::Float:
//         e.f = point.getFieldAs<float>(dim.m_id);
//         break;
//     case Dimension::Type::Signed8:
//         e.s8 = point.getFieldAs<int8_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed16:
//         e.s16 = point.getFieldAs<int16_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed32:
//         e.s32 = point.getFieldAs<int32_t>(dim.m_id);
//         break;
//     case Dimension::Type::Signed64:
//         e.s64 = point.getFieldAs<int64_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned8:
//         e.u8 = point.getFieldAs<uint8_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned16:
//         e.u16 = point.getFieldAs<uint16_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned32:
//         e.u32 = point.getFieldAs<uint32_t>(dim.m_id);
//         break;
//     case Dimension::Type::Unsigned64:
//         e.u64 = point.getFieldAs<uint64_t>(dim.m_id);
//         break;
//     default:
//         throw pdal_error("Unsupported attribute type for " + dim.m_name);
//     }
//
//     size_t size = Dimension::size(dim.m_type);
//     memcpy(dim.m_buffer.data() + (idx * size), &e, size);
// }


DracoWriter::DracoWriter():
    m_args(new DracoWriter::Args)
{
//     m_args->m_defaults = NL::json::parse(attributeDefaults);
}


DracoWriter::~DracoWriter(){}


std::string DracoWriter::getName() const { return s_info.name; }


void DracoWriter::addArgs(ProgramArgs& args)
{
    args.add("array_name", "Draco array name",
        m_args->m_arrayName).setPositional();
    args.addSynonym("array_name", "filename");
}


void DracoWriter::initialize()
{

}


void DracoWriter::ready(pdal::BasePointTable &table)
{
    auto layout = table.layout();
    auto all = layout->dims();

    m_current_idx = 0;
}


bool DracoWriter::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);



    return true;
}


void DracoWriter::write(const PointViewPtr view)
{
    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


void DracoWriter::done(PointTableRef table)
{


}



} // namespace pdal
