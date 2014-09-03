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

#pragma once

#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/pdal_defines.h>
#include <pdal/Bounds.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/Options.hpp>

namespace pdal
{

namespace utils
{

using namespace boost::property_tree;

inline ptree toPTree(MetadataNode const& node)
{
    typedef ptree::path_type path;

    ptree tree;
    tree.put("name", node.name());
    tree.put("description", node.description());
    tree.put("type", node.type());
    tree.put("value", node.value());

    for (auto n = node.children().begin(); n != node.children().end(); ++n)
    {

        ptree pnode = toPTree(*n);
        if (boost::algorithm::iequals(node.kind(),"array"))
        {
            boost::optional<ptree&> opt =
                tree.get_child_optional(path(node.name(), '/'));
            if (opt)
                opt->push_back(std::make_pair("", pnode));
            else
            {
                tree.push_back(ptree::value_type(node.name(), ptree()));
                auto& p = tree.get_child(path(node.name(), '/'));
                p.push_back(std::make_pair("", pnode));

            }
        }
        else if (node.name().size())
            tree.push_back(std::make_pair(node.name(), pnode));
    }
    return tree;
}


inline ptree toPTree(const PointContext& ctx)
{
    ptree tree;
    ptree dimsTree;

    for (const auto& id : ctx.dims())
    {
        ptree dim;
        dim.put("name", Dimension::name(id));
        Dimension::Type::Enum t = ctx.dimType(id);
        dim.put("type", Dimension::toName(Dimension::base(t)));
        dim.put("size", ctx.dimSize(id));
        dimsTree.push_back(std::make_pair("", dim));
    }

    tree.add_child("dimensions", dimsTree);

    return tree;
}


/*! returns a boost::property_tree containing the point records, which is
    useful for dumping and such.
    \verbatim embed:rst
    ::

        0:
            X: 1.00
            Y: 2.00
            Z: 3.00
        1:
            X: 1.00
            Y: 2.00
            Z: 3.00

    \endverbatim
*/
inline ptree toPTree(const PointBuffer& buffer)
{

    ptree tree;

    const Dimension::IdList& dims = buffer.dims();

    for (PointId idx = 0; idx < buffer.size(); idx++)
    {
        std::string pointstring = boost::lexical_cast<std::string>(idx) + ".";

        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            std::string key = pointstring + Dimension::name(*di);
            double v = buffer.getFieldAs<double>(*di, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            tree.add(key, value);
        }
    }

    return tree;
}


template <typename T> ptree toPTree(const Range<T>& rng)
{
    ptree tree;
    tree.add("minimum", rng.getMinimum());
    tree.add("maximum", rng.getMaximum());
    return tree;
}


/// Outputs a string-based boost::property_tree::ptree representation
/// of the Bounds instance
template <typename T> ptree toPTree(const Bounds<T>& bounds)
{
    ptree tree;
    for (std::size_t i = 0; i < bounds.size(); ++i)
    {
        const Range<T>& r = bounds.dimensions()[i];
        tree.add_child(boost::lexical_cast<std::string>(i), toPTree(r));
    }
    return tree;
}

ptree toPTree(const Options& options);
inline ptree toPTree(const Option& option)
{
    ptree t;
    t.put("Name", option.getName());
    t.put("Value", option.getValue<std::string>());
    if (option.getDescription() != "")
    {
        t.put("Description", option.getDescription());
    }
    boost::optional<Options const&> options = option.getOptions();

    if (options != boost::none)
    {
        t.add_child("Options", toPTree(*options));
    }
    return t;
}


inline ptree toPTree(const Options& options)
{
    ptree tree;
    std::vector<Option> opts = options.getOptions();
    for (auto citer = opts.begin(); citer != opts.end(); ++citer)
    {
        const Option& option = *citer;
        ptree subtree = toPTree(option);
        tree.add_child("Option", subtree);
    }
    return tree;
}



inline ptree toPTree(const SpatialReference& ref)
{
    ptree srs;

#ifdef PDAL_SRS_ENABLED
    srs.put("proj4", ref.getProj4());
    srs.put("prettywkt", ref.getWKT(SpatialReference::eHorizontalOnly, true));
    srs.put("wkt", ref.getWKT(SpatialReference::eHorizontalOnly, false));
    srs.put("compoundwkt", ref.getWKT(SpatialReference::eCompoundOK, false));
    srs.put("prettycompoundwkt", ref.getWKT(SpatialReference::eCompoundOK, true));
#else
    std::string message;
    std::string wkt = ref.getWKT();
    if (wkt.size() == 0)
        message = "Reference defined with VLR keys, but GeoTIFF and GDAL "
            "support are not available to produce definition";
    else if (wkt.size() > 0)
        message = "Reference defined with WKT, but GeoTIFF and GDAL "
            "support are not available to produce definition";
    else
        message = "None";

    srs.put("proj4", message);
    srs.put("prettywkt", message);
    srs.put("wkt", message);
    srs.put("compoundwkt", message);
    srs.put("prettycompoundwkt", message);
    srs.put("gtiff", message);
#endif
    return srs;
}


namespace reST
{


inline std::ostream& toRST(const PointBuffer& buffer, std::ostream& os)
{
    const Dimension::IdList& dims = buffer.dims();

    size_t name_column(20);
    size_t value_column(40);

    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        std::string name = Dimension::name(*di);
        name_column = std::max(name_column, name.size());
    }

    std::ostringstream thdr;
    for (unsigned i = 0; i < name_column - 1; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < value_column - 1; ++i)
        thdr << "=";
    thdr << " ";
    thdr << " ";

    name_column--;
    unsigned step_back(3);

    std::string hdr(80, '-');
    for (PointId idx = 0; idx < buffer.size(); ++idx)
    {
        os << "Point " << idx << std::endl;
        os << hdr << std::endl << std::endl;
        os << thdr.str() << std::endl;
        os << std::setw(name_column-step_back) << "Name" <<
            std::setw(value_column-step_back) << "Value"  << std::endl;
        os << thdr.str() << std::endl;
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            double v = buffer.getFieldAs<double>(*di, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            os << std::left << std::setw(name_column) << Dimension::name(*di) <<
                std::right << std::setw(value_column) << value << std::endl;
        }
        os << thdr.str() << std::endl << std::endl;
    }
    os << std::endl << std::endl;
    return os;
}


} // reST

} // utils
} // pdal
