/******************************************************************************
* Copyright (c) 2014,  Hobu Inc., hobu@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
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

#include <pdal/DbWriter.hpp>

namespace pdal
{

void DbWriter::ready(PointContextRef ctx)
{
    using namespace Dimension;

    m_pointSize = 0;
    // Temp.
    m_dimTypes = ctx.dimTypes();
    m_dims = ctx.dims();

    // If we find an X, Y or Z, remove it and stick it at the end.  This allows
    // the buffer that we build based on the dimensions to be modified from
    // source type (doubles) to 

    // Determine types for the dimensions.  We use the default types when
    // they exist, float otherwise.
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        Type::Enum type = ctx.dimType(*di);

        m_types.push_back(type);
        m_pointSize += size(type);
    }
}

/// Determine if X, Y and Z values should be written as Signed32 along with
/// a scale factor and offset instead of being written as Double.
///
/// \return  Whether X,Y and Z values should be scaled.
bool DbWriter::locationScaling() const
{
    return (m_xXform.nonstandard() || m_yXform.nonstandard() ||
        m_zXform.nonstandard());
}

/**
        if (id == Id::X || id == Id::Y || id == Id::Z && locationScaling())
            type = Type::Signed32;
**/

} // namespace pdal
