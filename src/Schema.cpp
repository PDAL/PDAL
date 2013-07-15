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

#include <iostream>
#include <map>

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


Schema::Schema()
    : m_byteSize(0)
{
    return;
}

Schema::Schema(std::vector<Dimension> const& dimensions)
    : m_byteSize(0)
{

    for (std::vector<Dimension>::const_iterator i = dimensions.begin();
            i != dimensions.end(); ++i)
    {
        appendDimension(*i);
    }
}

/// copy constructor
Schema::Schema(Schema const& other)
    : m_byteSize(other.m_byteSize)
    , m_index(other.m_index)
{

}


// assignment constructor
Schema& Schema::operator=(Schema const& rhs)
{
    if (&rhs != this)
    {
        m_byteSize = rhs.m_byteSize;
        m_index = rhs.m_index;
    }

    return *this;
}


bool Schema::operator==(const Schema& other) const
{
    if (m_byteSize != other.m_byteSize) return false;

    if (m_index.size() != other.m_index.size()) return false;

    schema::index_by_index const& idx = m_index.get<schema::index>();
    schema::index_by_index const& idx2 = m_index.get<schema::index>();

    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        if (!(idx[i] == idx2[i]))
        {
            return false;
        }
    }

    return true;
}


bool Schema::operator!=(const Schema& other) const
{
    return !(*this==other);
}


void Schema::appendDimension(const Dimension& dim)
{
    // Copy the Dimension because we're going to overwrite/set some of
    // its attributes
    Dimension d(dim);

    schema::index_by_position& position_index = m_index.get<schema::position>();

    schema::index_by_position::const_reverse_iterator r = position_index.rbegin();

    // If we do not have anything in the index, set our current values.
    // Otherwise, use the values from the last entry in the index as our
    // starting point
    if (r != position_index.rend())
    {
        Dimension const& t = *r;

        m_byteSize = m_byteSize + d.getByteSize();

        d.setByteOffset(t.getByteOffset() + t.getByteSize());
    }
    else
    {
        m_byteSize = d.getByteSize();
        d.setByteOffset(0);
    }

    d.setPosition(m_index.size());

    schema::index_by_uid const& id_index = m_index.get<schema::uid>();
    schema::index_by_uid::size_type id_count = id_index.count(d.getUUID());
    
    if (id_count >0)
    {
        std::ostringstream oss;
        oss << "Dimension with uuid '" << d.getUUID() << "' already exists with name '" << d.getName() << "' and namespace '" << d.getNamespace() << "'";
        throw duplicate_dimension_id(oss.str());
    }
    std::pair<schema::index_by_position::iterator, bool> q = position_index.insert(d);
    if (!q.second)
    {
        std::ostringstream oss;
        oss << "Could not insert into schema index because of " << q.first->getName();
        throw schema_error(oss.str());
    }

    return;
}


const Dimension& Schema::getDimension( schema::size_type t) const
{
    schema::index_by_index const& idx = m_index.get<schema::index>();

    if (t >= idx.size())
        throw dimension_not_found("Index position is not valid");

    return idx.at(t);
}

boost::optional<Dimension const&> Schema::getDimensionOptional(schema::size_type t) const
{
    try
    {
        Dimension const& dim = getDimension(t);
        return boost::optional<Dimension const&>(dim);
    }
    catch (pdal::dimension_not_found&)
    {
        return boost::optional<Dimension const&>();
    }
}


const Dimension& Schema::getDimension(boost::string_ref name, boost::string_ref namespc) const
{
    std::string errorMsg;
    const Dimension* dim = getDimensionPtr(name, namespc, &errorMsg);
    if (!dim)
    {
        throw dimension_not_found(errorMsg);
    }
    return *dim;
}


namespace {
// Helpers for searching the dimension index without constructing a std::string

// Hash for boost::string_ref
struct string_ref_hash
{
    std::size_t operator()(boost::string_ref str) const
    {
        return boost::hash_range(str.begin(), str.end());
    }
};

// Compare boost::string_ref and std::string
struct string_ref_equal
{
    template<typename T1, typename T2>
    bool operator()(const T1& s1, const T2& s2) const
    {
        return s1 == s2;
    }
};

} // namespace


const Dimension* Schema::getDimensionPtr(boost::string_ref name, boost::string_ref namespc,
                                         std::string* errorMsg) const
{
    // getDimensionPtr is implemented in terms of boost::string_ref so that we
    // can guarentee not to allocate memory unless we really need to.  That
    // makes the following code ugly but significantly faster when performing a
    // lot of dimension lookup.
    boost::string_ref t = name;
    boost::string_ref ns = namespc;
    if (ns.empty())
    {
        size_t dotPos = name.rfind('.');
        if (dotPos != boost::string_ref::npos)
        {
            // dimension is named as namespace.name (eg, drivers.las.reader.X)
            // - split into name and namespace
            t = name.substr(dotPos + 1);
            ns = name.substr(0, dotPos);
        }
    }

    schema::index_by_name const& name_index = m_index.get<schema::name>();
    std::pair<schema::index_by_name::const_iterator,
              schema::index_by_name::const_iterator> nameRange =
        name_index.equal_range(t, string_ref_hash(), string_ref_equal());

    if (nameRange.first != name_index.end())
    {
        if (ns.empty())
        {
            for (schema::index_by_name::const_iterator it = nameRange.first;
                 it != nameRange.second; ++it)
            {
                if (ns == it->getNamespace())
                {
                    return &*it;
                }
            }
            // Name found, but requested namespace not found
            if (errorMsg)
            {
                std::ostringstream oss;
                oss << "Unable to find dimension with name '" << t
                    << "' and namespace  '" << ns << "' in schema";
                *errorMsg = oss.str();
            }
            return 0;
        }

        // Determine whether we have more than one dimension of the same name
        schema::index_by_name::const_iterator it = nameRange.first;
        ++it;
        if (it != nameRange.second)
        {
            boost::uint32_t num_parents(0);
            std::map<dimension::id, dimension::id> relationships;

            for (schema::index_by_name::const_iterator  o = nameRange.first; o != nameRange.second; ++o)
            {
                // Put a map together that maps parents to children that
                // we are going to walk to find the very last child in the
                // graph.
                std::pair<dimension::id, dimension::id> p(o->getParent(), o->getUUID());
                relationships.insert(p);

                // The parent dimension should have a nil parent of its own.
                // nil_uuid is the default parent of all dimensions as the y
                // are created
                if (o->getParent().is_nil())
                {
                    num_parents++;
                }
            }

            // Test to make sure that the number of parent dimensions all with
            // the same name is equal to only 1. If there are multiple
            // dimensions with the same name, but no relationships defined,
            // we are in an error condition
            if (num_parents != 1)
            {
                if (errorMsg)
                {
                    std::ostringstream oss;
                    oss << "Schema has multiple dimensions with name '" << t << "', but "
                        "their parent/child relationships are not coherent. Multiple "
                        "parents are present.";
                    *errorMsg = oss.str();
                }
                return 0;
            }

            dimension::id parent = boost::uuids::nil_uuid();

            // Starting at the parent (nil uuid), walk the child/parent graph down to the
            // end.  When we're done finding dimensions, what's left is the child
            // at the end of the graph.
            std::map<dimension::id, dimension::id>::const_iterator p = relationships.find(parent);
            dimension::id child;
            while (p != relationships.end())
            {
                child = p->second;
                p = relationships.find(p->second);
            }
            schema::index_by_uid::const_iterator pi = m_index.get<schema::uid>().find(child);
            if (pi != m_index.get<schema::uid>().end())
            {
                return &*pi;
            }
            else
            {
                if (errorMsg)
                {
                    std::ostringstream oss;
                    oss << "Unable to fetch subjugate dimension with id '" << child << "' in schema";
                    *errorMsg = oss.str();
                }
                return 0;
            }
        }
        // we don't have a ns or we don't have multiples, return what we found
        return &*nameRange.first;
    }
    else
    {
        // FIXME!!!  uuids::string_generator will generally throw - need our own parsing function.
        boost::uuids::uuid ps1;
        try
        {
            boost::uuids::string_generator gen;
            ps1 = gen(t.begin(), t.end());
        }
        catch (std::runtime_error&)
        {
            // invalid string for uuid
            if (errorMsg)
            {
                std::ostringstream oss;
                oss << "Dimension with name '" << t << "' not found, unable to Schema::getDimension";
                *errorMsg = oss.str();
            }
            return 0;
        }

        schema::index_by_uid::const_iterator i = m_index.get<schema::uid>().find(ps1);

        if (i != m_index.get<schema::uid>().end())
        {
            if (!ns.empty())
            {
                while (i != m_index.get<schema::uid>().end())
                {
                    if (ns == i->getNamespace())
                    {
                        return &*i;
                    }
                    ++i;
                }
            }
            return &*i;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}


boost::optional<Dimension const&> Schema::getDimensionOptional(std::string const& name, std::string const& ns) const
{
    const Dimension* dim = getDimensionPtr(name.c_str(), ns.c_str());
    if (dim)
        return boost::optional<Dimension const&>(*dim);
    else
        return boost::optional<Dimension const&>();
}

bool Schema::setDimension(Dimension const& dim)
{
    // Try setting based on UUID first if it's there and not null.
    if (dim.getUUID() != boost::uuids::nil_uuid())
    {
        schema::index_by_uid& id_index = m_index.get<schema::uid>();
        schema::index_by_uid::const_iterator id = id_index.find(dim.getUUID());
        if (id != id_index.end())
        {
            id_index.replace(id, dim);
            return true;
        }
    }

    schema::index_by_name& name_index = m_index.get<schema::name>();
    schema::index_by_name::iterator it = name_index.find(dim.getName());
    // FIXME: If there are two dimensions with the same name here, we're
    // screwed if they both have the same namespace too
    if (it != name_index.end())
    {
        while (it != name_index.end())
        {
            if (boost::equals(dim.getNamespace(), it->getNamespace()))
            {
                name_index.replace(it, dim);
                return true;
            }
            ++it;
        }
    }
    else
    {
        std::ostringstream oss;
        oss << "Dimension with name '" << dim.getName() << "' not found, unable to Schema::setDimension";
        throw dimension_not_found(oss.str());
    }

    return true;
}

const Dimension& Schema::getDimension(dimension::id const& t) const
{
    /// getDimension for a dimension::id will not respect the isIgnored setting 
    /// of the dimension.  Stages that wish to operate with dimensions with exacting 
    /// specificity should take care to use uuids as their keys rather than 
    /// names.
    schema::index_by_uid::const_iterator it = m_index.get<schema::uid>().find(t);

    if (it != m_index.get<schema::uid>().end())
    {
        return *it;
    }

    std::ostringstream oss;
    oss << "getDimension: dimension not found with uuid '" << boost::lexical_cast<std::string>(t) << "'";
    throw dimension_not_found(oss.str());
}

boost::optional<Dimension const&> Schema::getDimensionOptional(dimension::id const& t) const
{
    try
    {
        Dimension const& dim = getDimension(t);
        return boost::optional<Dimension const&>(dim);
    }
    catch (pdal::dimension_not_found&)
    {
        return boost::optional<Dimension const&>();
    }
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
#ifdef PDAL_HAVE_LIBXML2

    std::string xsd("");

    pdal::schema::Reader reader(xml, xsd);

    pdal::Schema schema = reader.getSchema();
    return schema;

#else
    boost::ignore_unused_variable_warning(xml);
    return Schema();
#endif
}

std::string Schema::to_xml(Schema const& schema, boost::property_tree::ptree const* metadata)
{
#ifdef PDAL_HAVE_LIBXML2

    pdal::schema::Writer writer(schema);
    
    if (metadata)
        writer.setMetadata(*metadata);

    return writer.getXML();

#else
    boost::ignore_unused_variable_warning(schema);
    return std::string("");
#endif
}


boost::property_tree::ptree Schema::toPTree() const
{
    boost::property_tree::ptree tree;

    schema::index_by_index const& idx = m_index.get<schema::index>();

    for (schema::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;
        tree.add_child("dimension", dim.toPTree());
    }

    return tree;
}


void Schema::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, pdal::Schema const& schema)
{
    boost::property_tree::ptree tree = schema.toPTree();

    boost::property_tree::write_json(os, tree);

    return os;
}


} // namespace pdal
