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

#include <pdal/Metadata.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/pdal_defines.h>
#include <pdal/pdal_export.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>

namespace pdal
{

namespace utils
{

inline void printError(const std::string& s)
{
    std::cerr << "PDAL: " << s << std::endl;
    std::cerr << std::endl;
}

using namespace boost::property_tree;

inline ptree toPTree(MetadataNode const& node)
{
    typedef ptree::path_type path;

    ptree tree;
    tree.put("name", node.name());
    tree.put("description", node.description());
    tree.put("type", node.type());
    tree.put("value", node.value());

    MetadataNodeList children = node.children();
    for (auto n = children.begin(); n != children.end(); ++n)
    {
        ptree pnode = toPTree(*n);
        if (node.kind() == MetadataType::Array)
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


inline MetadataNode toMetadata(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    MetadataNode root;

    for (const auto& id : layout->dims())
    {
        MetadataNode dim("dimensions");
        dim.add("name", layout->dimName(id));
        Dimension::Type::Enum t = layout->dimType(id);
        dim.add("type", Dimension::toName(Dimension::base(t)));
        dim.add("size", layout->dimSize(id));
        root.addList(dim);
    }

    return root;
}


inline MetadataNode toMetadata(const PointViewPtr view)
{
    MetadataNode node;

    const Dimension::IdList& dims = view->dims();

    for (PointId idx = 0; idx < view->size(); idx++)
    {
        MetadataNode pointnode = node.add(std::to_string(idx));
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            double v = view->getFieldAs<double>(*di, idx);
            pointnode.add(Dimension::name(*di), v);
        }
    }

    return node;
}

/// Outputs a string-based boost::property_tree::ptree representation
/// of the BOX3D instance
inline ptree toPTree(const BOX3D& bounds)
{
    ptree tree;
    ptree x;
    ptree y;
    ptree z;

    x.add("minimum", bounds.minx);
    x.add("maximum", bounds.maxx);
    tree.add_child("0", x);

    y.add("minimum", bounds.miny);
    y.add("maximum", bounds.maxy);
    tree.add_child("1", y);

    if (!bounds.is_z_empty())
    {
        z.add("minimum", bounds.minz);
        z.add("maximum", bounds.maxz);
        tree.add_child("2", z);
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

    srs.put("proj4", ref.getProj4());
    srs.put("prettywkt", ref.getWKT(SpatialReference::eHorizontalOnly, true));
    srs.put("wkt", ref.getWKT(SpatialReference::eHorizontalOnly, false));
    srs.put("compoundwkt", ref.getWKT(SpatialReference::eCompoundOK, false));
    srs.put("prettycompoundwkt", ref.getWKT(SpatialReference::eCompoundOK, true));

    return srs;
}

std::string PDAL_DLL toJSON(const MetadataNode& m);
void PDAL_DLL toJSON(const MetadataNode& m, std::ostream& o);
std::string PDAL_DLL toJSON(const Options& opts);
void PDAL_DLL toJSON(const Options& opts, std::ostream& o);

namespace reST
{

std::ostream& toRST(const ptree&, std::ostream& os);

void PDAL_DLL write_rst(std::ostream& ost,
               const boost::property_tree::ptree& pt,
               int level=0);


inline std::ostream& toRST(const PointViewPtr view, std::ostream& os)
{
    const Dimension::IdList& dims = view->dims();

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
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        os << "Point " << idx << std::endl;
        os << hdr << std::endl << std::endl;
        os << thdr.str() << std::endl;
        os << std::setw(name_column-step_back) << "Name" <<
            std::setw(value_column-step_back) << "Value"  << std::endl;
        os << thdr.str() << std::endl;
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            double v = view->getFieldAs<double>(*di, idx);
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
