/******************************************************************************
 * Copyright (c) 2012, Howard Butler (howard@hobu.co)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/Metadata.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

template <>
void MetadataNodeImpl::setValue(const SpatialReference& ref)
{
    m_type = "spatialreference";
    m_value = Utils::toString(ref);
}


std::string Metadata::inferType(const std::string& val)
{
    size_t pos;

    long l = 0;
    try
    {
        pos = 0;
        l = std::stol(val, &pos);
    }
    catch (std::invalid_argument&)
    {}
    if (pos == val.length())
        return (l < 0 ? "nonNegativeInteger" : "integer");

    try
    {
        pos = 0;

        // silence discarding return value of function with 'nodiscard' attribute
        // with the (void) cast
        (void)std::stod(val, &pos);
    }
    catch(std::invalid_argument&)
    {}

    if (pos == val.length())
        return "double";

    BOX2D b2d;
    std::istringstream iss1(val);
    try
    {
        iss1 >> b2d;
        if (iss1.good())
            return "bounds";
    }
    catch (const BOX2D::error&)
    {}

    BOX3D b3d;
    std::istringstream iss2(val);
    try
    {
        iss2 >> b3d;
        if (iss2.good())
            return "bounds";
    }
    catch (const BOX3D::error&)
    {}

    if (val == "true" || val == "false")
        return "boolean";

    try
    {
        SpatialReference s(val);
        return "spatialreference";
    }
    catch (pdal_error&)
    {
    }

    Uuid uuid(val);
    if (!uuid.isNull())
        return "uuid";

    return "string";
}


} // namespace pdal
