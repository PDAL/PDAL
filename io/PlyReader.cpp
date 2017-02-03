/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include "PlyReader.hpp"

#include <sstream>

#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{
namespace
{

struct CallbackContext
{
    PointViewPtr view;
    PlyReader::DimensionMap dimensionMap;
};


void plyErrorCallback(p_ply ply, const char * message)
{
    throw PlyReader::error(message);
}


p_ply openPly(std::string filename)
{
    p_ply ply = ply_open(filename.c_str(), &plyErrorCallback, 0, nullptr);
    if (!ply)
        throw PlyReader::error("Unable to open file " + filename +
            " for reading.");

    if (!ply_read_header(ply))
        throw PlyReader::error("Unable to read header of " + filename + ".");
    return ply;
}


int readPlyCallback(p_ply_argument argument)
{
    p_ply_element element;
    long index;
    void * contextAsVoid;
    long numToRead;
    p_ply_property property;
    const char * propertyName;

    if (!ply_get_argument_element(argument, &element, &index))
        throw PlyReader::error("Error getting argument element.");

    if (!ply_get_argument_user_data(argument, &contextAsVoid, &numToRead))
        throw PlyReader::error("Error getting argument user data.");

    // We've read enough, abort the callback cycle
    if (numToRead <= index)
        return 0;

    if (!ply_get_argument_property(argument, &property, nullptr, nullptr))
        throw PlyReader::error("Error getting argument property.");

    if (!ply_get_property_info(property, &propertyName, nullptr,
        nullptr, nullptr))
        throw PlyReader::error("Error getting property info.");

    CallbackContext * context = static_cast<CallbackContext *>(contextAsVoid);
    double value = ply_get_argument_value(argument);
    Dimension::Id dimension = context->dimensionMap.at(propertyName);
    context->view->setField(dimension, index, value);

    return 1;
}

} // unnamed namespace


static PluginInfo const s_info = PluginInfo(
        "readers.ply",
        "Read ply files.",
        "http://pdal.io/stages/reader.ply.html");


CREATE_STATIC_PLUGIN(1, 0, PlyReader, Reader, s_info);


std::string PlyReader::getName() const
{
    return s_info.name;
}


PlyReader::PlyReader()
    : m_ply(nullptr)
    , m_vertexDimensions()
{}


void PlyReader::initialize()
{
    try
    {
        p_ply ply = openPly(m_filename);
        p_ply_element vertex_element = nullptr;
        bool found_vertex_element = false;
        const char* element_name;
        long element_count;
        while ((vertex_element = ply_get_next_element(ply, vertex_element)))
        {
            if (!ply_get_element_info(vertex_element, &element_name,
                        &element_count))
                throwError("Error reading element info in " + m_filename + ".");
            if (strcmp(element_name, "vertex") == 0)
            {
                found_vertex_element = true;
                break;
            }
        }
        if (!found_vertex_element)
            throwError("File " + m_filename + " does not contain a vertex "
                    "element.");

        static std::map<int, Dimension::Type> types =
        {
            { PLY_INT8, Dimension::Type::Signed8 },
            { PLY_UINT8, Dimension::Type::Unsigned8 },
            { PLY_INT16, Dimension::Type::Signed16 },
            { PLY_UINT16, Dimension::Type::Unsigned16 },
            { PLY_INT32, Dimension::Type::Signed32 },
            { PLY_UINT32, Dimension::Type::Unsigned32 },
            { PLY_FLOAT32, Dimension::Type::Float },
            { PLY_FLOAT64, Dimension::Type::Double },

            { PLY_CHAR, Dimension::Type::Signed8 },
            { PLY_UCHAR, Dimension::Type::Unsigned8 },
            { PLY_SHORT, Dimension::Type::Signed16 },
            { PLY_USHORT, Dimension::Type::Unsigned16 },
            { PLY_INT, Dimension::Type::Signed32 },
            { PLY_UINT, Dimension::Type::Unsigned32 },
            { PLY_FLOAT, Dimension::Type::Float },
            { PLY_DOUBLE, Dimension::Type::Double }
        };

        p_ply_property property = nullptr;
        while ((property = ply_get_next_property(vertex_element, property)))
        {
            const char* name;
            e_ply_type type;
            e_ply_type length_type;
            e_ply_type value_type;
            if (!ply_get_property_info(property, &name, &type,
                        &length_type, &value_type))
                throwError("Error reading property info in " +
                    m_filename + ".");
            // For now, we'll just use PDAL's built in dimension matching.
            // We could be smarter about this, e.g. by using the length
            // and value type attributes.
            m_vertexTypes[name] = types[type];
        }
        ply_close(ply);
    }
    catch (const error& err)
    {
        throwError(err.what());
    }
}


void PlyReader::addDimensions(PointLayoutPtr layout)
{
    for (auto it : m_vertexTypes)
    {
        const std::string& name = it.first;
        const Dimension::Type& type = it.second;

        m_vertexDimensions[name] = layout->registerOrAssignDim(name, type);
    }
}


void PlyReader::ready(PointTableRef table)
{
    try
    {
        m_ply = openPly(m_filename);
    }
    catch (const error& err)
    {
        throwError(err.what());
    }
}


point_count_t PlyReader::read(PointViewPtr view, point_count_t num)
{
    CallbackContext context;
    context.view = view;
    context.dimensionMap = m_vertexDimensions;

    // It's possible that point_count_t holds a value that's larger than the
    // long that is the maximum rply (don't know about ply) point count.
    long cnt;
    cnt = Utils::inRange<long>(num) ? num : (std::numeric_limits<long>::max)();
    try
    {
        for (auto it : m_vertexDimensions)
        {
            ply_set_read_cb(m_ply, "vertex", it.first.c_str(), readPlyCallback,
                    &context, cnt);
        }
        if (!ply_read(m_ply))
        {
            throwError("Error reading " + m_filename + ".");
        }
    }
    catch(const error& err)
    {
        throwError(err.what());
    }
    return view->size();
}


void PlyReader::done(PointTableRef table)
{
    try
    {
        if (!ply_close(m_ply))
            throwError("Error closing " + m_filename + ".");
    }
    catch (const error& err)
    {
        throwError(err.what());
    }
}

} // namespace pdal
