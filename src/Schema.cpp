/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Schema implementation for C++ libLAS
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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

#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <map>
#include <algorithm>

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/uuid/uuid_io.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/algorithm/string.hpp>

#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

namespace pdal
{

bool Schema::operator==(const Schema& other) const
{
    if (m_fqIndex.size() != other.m_fqIndex.size())
        return false;
    std::map<std::string, DimensionPtr> idx(
        m_fqIndex.begin(), m_fqIndex.end());
    std::map<std::string, DimensionPtr> oidx(
        other.m_fqIndex.begin(), other.m_fqIndex.end());

    auto oi = oidx.begin();
    for (auto ii = idx.begin(); ii != idx.end(); ++ii, ++oi)
        if (ii->first != oi->first || ii->second != oi->second)
            return false;
    return true;
}


bool Schema::operator!=(const Schema& other) const
{
    return !(*this==other);
}


void Schema::appendDimension(const Dimension& dim)
{
    std::string name = dim.getName();
    std::string fqName = dim.getFQName();

    DimensionPtr d(new Dimension(dim));
    // If we already have a dimension with this fully-qualified name, don't
    // do anything other than make sure the dimensions match.
    auto di = m_fqIndex.find(fqName);
    if (di != m_fqIndex.end())
    {
        DimensionPtr tempDim = di->second;
        if (*tempDim != *d)
            throw pdal_error("Attempted to add unequal dimensions with "
                "equivalent fully-qualified names (namespace.name)");
        return;
    }
    m_fqIndex[fqName] = d;

    // If a dimension with this name doesn't already exist or we are going
    // to keep a secondary dimension with the same name, add it to the
    // position list so that we keep things in an order something like
    // we used to use.
    if (m_index.find(name) != m_index.end() || d->forceKeep())
        m_dimPos.push_back(d);
    std::cerr << "Adding dimension = " << name << " = " << d->getName() << "!\n";
    m_index[name] = d;
}


DimensionPtr Schema::getDimension(const std::string& name) const
{
    auto di = m_index.find(name);
//    return (di == m_index.end() ? DimensionPtr() : di->second);
    if (di == m_index.end())
    {
        std::cerr << "Returning null ptr for " << name << "!\n";
        return DimensionPtr();
    }
    else
    {
        std::cerr << "Returning dim = " << di->second->getName() << " for " <<
            name << "!\n";
        return di->second;
    }
}


DimensionPtr Schema::getDimension(const std::string& name,
    const std::string& nsName) const
{
    std::string fqName = name + "." + nsName;
    auto di = m_index.find(fqName);
    return (di == m_index.end() ? DimensionPtr() : di->second);
}


DimensionList Schema::getDimensions(const std::string& nsName) const
{
    DimensionList dims;
    for (auto di = m_dimPos.begin(); di != m_dimPos.end(); ++di)
    {
        DimensionPtr d = *di;
        if (d->getNamespace() == nsName)
            dims.push_back(d);
    }
    return dims;
}


void Schema::dump() const
{
    std::cout << *this;
}


Schema Schema::from_xml(std::string const& xml, std::string const& xsd)
{
#ifdef PDAL_HAVE_LIBXML2

    pdal::schema::Reader reader(xml, xsd);

    pdal::Schema schema = reader.getSchema();
    return schema;

#else
    boost::ignore_unused_variable_warning(xml);
    boost::ignore_unused_variable_warning(xsd);
    return Schema();
#endif
}

Schema Schema::from_xml(std::string const& xml)
{
    Schema schema;
#ifdef PDAL_HAVE_LIBXML2
    schema::Reader reader(xml, "");
    schema = reader.getSchema();
#else
    boost::ignore_unused_variable_warning(xml);
#endif
    return schema;
}

std::string Schema::to_xml(Schema const& schema, MetadataNode m)
{
    std::string xml;
#ifdef PDAL_HAVE_LIBXML2
    pdal::schema::Writer writer(schema);
    writer.setMetadata(m);
    xml = writer.getXML();
#else
    boost::ignore_unused_variable_warning(schema);
#endif
    return xml;
}


boost::property_tree::ptree Schema::toPTree() const
{
    boost::property_tree::ptree tree;

    for (auto di = m_dimPos.begin(); di != m_dimPos.end(); ++di)
    {
        DimensionPtr d = *di;
        tree.add_child("dimension", d->toPTree());
    }
    return tree;
}


std::ostream& Schema::toRST(std::ostream& os) const
{
    uint32_t gap(1);
    uint32_t name_column(20 + gap);
    uint32_t scale_column(10 + gap);
    uint32_t offset_column(10 + gap);
    uint32_t type_column(9 + gap);
    uint32_t size_column(9 + gap);    

    for (auto di = m_dimPos.begin(); di != m_dimPos.end(); ++di)
    {
        DimensionPtr d = *di;

        name_column = std::max(name_column,
            (uint32_t)d->getFQName().size() + 2 * gap);
        scale_column = std::max(scale_column,
            (uint32_t)boost::lexical_cast<std::string>(
                d->getNumericScale()).size() + 2 * gap);
        offset_column = std::max(offset_column,
            (uint32_t)boost::lexical_cast<std::string>(
                d->getNumericOffset()).size() + 2 * gap);
        type_column = std::max(type_column,
            (uint32_t)d->getInterpretationName().size() + 2 * gap);
        size_column = std::max(size_column,
            (uint32_t)boost::lexical_cast<std::string>(
                d->getByteSize()).size() + 2 * gap);
    }

    std::ostringstream hdr;
    for (int i = 0; i < 80; ++i)
        hdr << "-";
    
    os << std::endl;
    os << "Schema" << std::endl;
    os << hdr.str() << std::endl << std::endl;
    
    std::string orientation("point");
    std::string size = boost::lexical_cast<std::string>(m_dimPos.size());
    
    os << "schema has " << size 
       << " dimensions and a total size of " << getByteSize() << " bytes. " <<
       std::endl << std::endl;
    
    std::ostringstream thdr;
    for (unsigned i = 0; i < name_column - gap; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < scale_column - gap; ++i)
        thdr << "=";
    thdr << " ";
    
    for (unsigned i = 0; i < offset_column - gap; ++i)
        thdr << "=";
    thdr << " ";
    
    for (unsigned i = 0; i < type_column - gap; ++i)
        thdr << "=";
    thdr << " ";

    for (unsigned i = 0; i < size_column - gap; ++i)
        thdr << "=";
    thdr << " ";
    
    name_column--;
    unsigned step_back(3);

    os << thdr.str() << std::endl;
    os << std::setw(name_column-step_back) << "Name" 
       << std::setw(scale_column) << "Scale"  
       << std::setw(offset_column) << "Offset" 
       << std::setw(type_column) << "Type" 
       << std::setw(size_column) << "Size" 
       << std::endl;
    os << thdr.str() << std::endl; 
    for (auto di = m_dimPos.begin(); di != m_dimPos.end(); ++di)
    {
        DimensionPtr d = *di;

        std::string scale(
            boost::lexical_cast<std::string>(d->getNumericScale()));
        std::string offset(
            boost::lexical_cast<std::string>(d->getNumericOffset()));
        std::string t(d->getInterpretationName());
        std::string size(
            boost::lexical_cast<std::string>(d->getByteSize()));

        os   << std::left << std::setw(name_column) << d->getFQName() <<
            std::right << std::setw(scale_column) << scale <<
            std::setw(offset_column) << offset <<
            std::setw(type_column) << t <<
            std::setw(size_column) << size << std::endl;
    }
    os << thdr.str() << std::endl << std::endl;
    
    for (auto di = m_dimPos.begin(); di != m_dimPos.end(); ++di)
    {
        DimensionPtr d = *di;

        std::ostringstream hdr;
        for (int i = 0; i < 80; ++i)
            hdr << ".";
        os << d->getFQName() << std::endl;
        os << hdr.str() << std::endl << std::endl;

        boost::uint32_t gap(1);
        boost::uint32_t key_column(32 + gap);
        boost::uint32_t value_column(40 + gap);        

        std::ostringstream thdr;
        for (unsigned i = 0; i < key_column - gap; ++i)
            thdr << "=";
        thdr << " ";

        for (unsigned i = 0; i < value_column - gap; ++i)
            thdr << "=";
        thdr << " ";

        os << thdr.str() << std::endl;
        os << std::left << std::setw(key_column-step_back) << "Name" 
           << std::right << std::setw(value_column-step_back) << "Value"  
           << std::endl;
        os << thdr.str() << std::endl;         
        key_column = key_column - gap;
        os << std::left << std::setw(key_column) << "Name" <<
            std::right << std::setw(value_column) << d->getName() << std::endl;        
        os << std::left << std::setw(key_column) << "Namespace" <<
            std::right << std::setw(value_column) << d->getNamespace() <<
            std::endl;        

        os << std::left << std::setw(key_column) << "ByteSize" <<
            std::right << std::setw(value_column) << d->getByteSize() <<
            std::endl;        

        os << std::left << std::setw(key_column) << "Ignored?" <<
            std::right << std::setw(value_column) << d->isIgnored() <<
            std::endl;        

        os << std::left << std::setw(key_column) << "UUID" <<
            std::right << std::setw(value_column) << d->getUUID() << std::endl;        
        os << std::left << std::setw(key_column) << "Scale" <<
            std::right << std::setw(value_column) << d->getNumericScale() <<
            std::endl;        

        os << std::left << std::setw(key_column) << "Offset" <<
            std::right << std::setw(value_column) << d->getNumericOffset() <<
            std::endl;        

        std::vector<std::string> lines;
        std::string description =
            boost::algorithm::erase_all_copy(d->getDescription(), "\n");
        
        Utils::wordWrap(description, lines, value_column - gap);
        os << std::left << std::setw(key_column) << "Description" << " ";
        if (lines.size() == 1)
        {
            os << std::right << std::setw(value_column - 1) << description <<
                std::endl;
        }
        else
        {
            os << std::setw(value_column) << lines[0] << std::endl;
        
            std::stringstream blank;
            size_t blanks(key_column + gap);
            for (size_t i = 0; i < blanks; ++i)
                blank << " ";
            for (size_t i = 1; i < lines.size(); ++i)
                os << std::left  << std::setw(blanks) << blank.str() <<
                    std::setw(value_column) << lines[i] << std::endl;
        }
        os << thdr.str() << std::endl << std::endl;
    }
    return os;
}


std::ostream& operator<<(std::ostream& os, pdal::Schema const& schema)
{
    boost::property_tree::ptree tree = schema.toPTree();
    boost::property_tree::write_json(os, tree);
    return os;
}

} // namespace pdal
