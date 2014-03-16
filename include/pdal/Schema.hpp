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

#ifndef PDAL_SCHEMA_HPP_INCLUDED
#define PDAL_SCHEMA_HPP_INCLUDED

#include <vector>
#include <map>

#include <pdal/pdal_internal.hpp>
#include <pdal/Dimension.hpp>

// boost
#include <boost/cstdint.hpp>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <boost/optional.hpp>

#include <pdal/third/string_ref.hpp>


#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/functional/hash.hpp>

namespace pdal
{

namespace schema
{
struct name {};
struct position {};
struct index {};
struct uid {};

typedef boost::multi_index::multi_index_container<
Dimension,
boost::multi_index::indexed_by<
// sort by Dimension::operator<
boost::multi_index::ordered_non_unique<boost::multi_index::tag<position>, boost::multi_index::identity<Dimension> >,

// Random access
boost::multi_index::random_access<boost::multi_index::tag<index> >,
// sort by less<string> on GetName
boost::multi_index::hashed_non_unique<boost::multi_index::tag<name>, boost::multi_index::const_mem_fun<Dimension,std::string const&,&Dimension::getName> >,
boost::multi_index::hashed_non_unique<boost::multi_index::tag<uid>, boost::multi_index::const_mem_fun<Dimension,dimension::id const&,&Dimension::getUUID> >
>
> Map;

typedef Map::index<name>::type index_by_name;
typedef Map::index<position>::type index_by_position;
typedef Map::index<index>::type index_by_index;
typedef Map::index<uid>::type index_by_uid;

typedef boost::uint32_t size_type;

class PDAL_DLL DimensionMap
{
public:
    inline void insert(std::pair<Dimension const*, Dimension const*> p) { m.insert(p); update();}
    
    std::map<Dimension const*, Dimension const*> m;
    
    static const int MAX_OFFSETS_LENGTH = 256;
    boost::uint64_t offsets[MAX_OFFSETS_LENGTH];

private:
    std::size_t update();
    
};


enum Orientation
{
    POINT_INTERLEAVED = 1,
    DIMENSION_INTERLEAVED = 2,
    UNKNOWN_INTERLEAVED = 256
};    

enum CompressionType
{
    COMPRESSION_NONE = 0,
    COMPRESSION_GHT = 1,
    COMPRESSION_DIMENSIONAL = 2,
    COMPRESSION_POINT = 4,
    COMPRESSION_LASZIP = 8,
    COMPRESSION_UNKNOWN = 256
};


}

/// A pdal::Schema is a composition of pdal::Dimension instances that form
/// a point cloud.
class PDAL_DLL Schema
{
public:

/// @name Constructors
    /// An empty constructor with no Dimension instances
    Schema();

    /// construct an instance given the order and dimensions in the dimensions
    /// vector @param dimensions the list of dimensions (and their order) to
    /// use to construct the Schema
    Schema(std::vector<Dimension> const& dimensions);

    /// Copy constructor
    Schema(Schema const& other);

    /// Assignment constructor
    Schema& operator=(Schema const& rhs);

/// @name Equality
    /// Equality
    bool operator==(const Schema& other) const;
    /// Inequality
    bool operator!=(const Schema& other) const;

/// @name Dimension manipulation
    /// adds (copies) a Dimension instance to the Schema
    /// @param dim a Dimension that is copied and added to the end of the Schema
    void appendDimension(Dimension const& dim);

    /*! overwrites an existing Dimension with the same name as dim
        \param dim the Dimension instance that contains the name and namespace
        to overwrite in the Schema.
        \verbatim embed:rst
        .. note::

            If no namespace is given, the *first* dimension with a matching
            :cpp:func:`pdal::Dimension::getName()` will be overwritten. To be
            sure, have set the namespace of the pdal::Dimension using
            :cpp:func:`pdal::Dimension::setNamespace()` beforehand.

        \endverbatim
    */
    bool setDimension(Dimension const& dim);

    /// @return the boost::multi_index map that contains the Dimension instances
    inline schema::Map const& getDimensions() const
    {
        return m_index;
    }

/// @name Dimension access
    /// @return a const& to a Dimension with the given name and namespace. If
    /// no matching dimension is found, pdal::dimension_not_found is thrown.
    /// @param name name to use when searching
    /// @param ns namespace to use when searching. If none is given, the first
    /// matching Dimension instance with name \b name is returned.
    const Dimension& getDimension(pdal::string_ref name,
                                  pdal::string_ref ns = string_ref()) const;

    /// @return a const& to a Dimension with the given dimension::id. If
    /// no matching dimension is found, pdal::dimension_not_found is thrown.
    /// @param id id to use when searching
    const Dimension& getDimension(dimension::id const& id) const;

    /// @return a const& to Dimension with the given index. If the
    /// index is out of range, pdal::dimension_not_found is thrown.
    /// @param index position index to return.
    const Dimension& getDimension(schema::size_type index) const;

    /// @return A pointer to a Dimension with the given name and namespace.  If
    /// no matching dimension is found, the null pointer is returned.  The
    /// optional errorMsg parameter will be filled with a descriptive error
    /// message in the case that the dimension cannot found.
    /// @param name name to use when searching
    /// @param ns namespace to use when searching. If none is given, the first
    /// matching Dimension instance with name \b name is returned.
    /// @param errorMsg optional location for storing error messages
    const Dimension* getDimensionPtr(string_ref name,
                                     string_ref ns = string_ref(),
                                     std::string* errorMsg = 0) const;

    /// @return a boost::optional-wrapped const& to a Dimension with the
    /// given name and namespace. If no matching dimension is found, the
    /// optional will be empty.
    /// @param name name to use when searching
    /// @param ns namespace to use when searching. If none is given, the first
    /// matching Dimension instance with name \b name is returned.
    boost::optional<Dimension const&> getDimensionOptional(string_ref name,
        string_ref ns=string_ref()) const;

    /// @return a boost::optional-wrapped const& to a Dimension with the
    /// given dimension::id.  If no matching dimension is found, the optional
    /// will be empty.
    /// @param id id to use when searching
    boost::optional<Dimension const&>
    getDimensionOptional(dimension::id const& id) const;

    /// @return a boost::optional-wrapped const& to a Dimension with the given
    /// index. If the index is out of range, the optional will be empty.
    /// @param index position index to return.
    boost::optional<Dimension const&>
    getDimensionOptional(schema::size_type index) const;

    /// @return the total cumulative byte size of all dimensions
    inline schema::size_type const& getByteSize() const
    {
        return m_byteSize;
    }

    /// @return the number of dimensions in this Schema instance
    inline schema::size_type size() const
    {
        return m_index.get<schema::name>().size();
    }

    inline schema::Orientation getOrientation() const
    {
        return m_orientation;
    }

    inline void setOrientation(schema::Orientation v)
    {
        m_orientation = v;
    }
    
    /// @return a new schema with all ignored fields removed.
    Schema pack() const;
    
/// @name Summary and serialization
    /// @return  a boost::property_tree representing the Schema
    /*!
        \verbatim embed:rst
        ::

            lo:
               dimension:
                   [Dimension ptree]
               dimension:
                   [Dimension ptree]

        \endverbatim
    */
    boost::property_tree::ptree toPTree() const;

    /// @return a schema::DimensionMap instance that maps dimension names
    schema::DimensionMap* mapDimensions(Schema const& destination,
        bool bIgnoreNamespace=false) const;

    std::ostream& toRST(std::ostream& os) const;
    
    /// dumps a string representation of the Schema instance to std::cout
    void dump() const;

    /// Deserialize a Schema instance from the given xml and validate
    /// against the given xsd
    /// @param xml xml data to ingest
    /// @param xsd xsd document to use for validation
    static Schema from_xml(std::string const& xml, std::string const& xsd);

    /// Deserialize a Schema instance from the given xml
    /// @param xml xml data to ingest
    static Schema from_xml(std::string const& xml);

    /// @return serialized Schema instance as xml
    static std::string to_xml(Schema const& schema,
        boost::property_tree::ptree const* metadata=0);
    
/// @name Private Attributes
private:

    schema::size_type m_byteSize;
    schema::Map m_index;
    schema::Orientation m_orientation;
};

PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Schema const& d);

} // namespace pdal

#endif // PDAL_SCHEMA_HPP_INCLUDED
