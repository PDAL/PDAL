/******************************************************************************
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

#pragma once

#include <unordered_map>
#include <vector>

#include <pdal/pdal_internal.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Metadata.hpp>

namespace pdal
{

<<<<<<< HEAD
=======
namespace schema
{
struct name {};
struct position {};
struct index {};
struct uid {};

typedef boost::multi_index::multi_index_container
<
    Dimension,
    boost::multi_index::indexed_by
    <
        // sort by Dimension::operator<
        boost::multi_index::ordered_non_unique<boost::multi_index::tag<position>,
            boost::multi_index::identity<Dimension> >,

        // Random access
        boost::multi_index::random_access<boost::multi_index::tag<index> >,

        // sort by less<string> on GetName
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<name>,
            boost::multi_index::const_mem_fun<Dimension,std::string const&,&Dimension::getName> >,

        // sort by less on UUID
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<uid>,
            boost::multi_index::const_mem_fun<Dimension,dimension::id const&,&Dimension::getUUID> >
    >
> Map;

typedef Map::index<name>::type index_by_name;
typedef Map::index<position>::type index_by_position;
typedef Map::index<index>::type index_by_index;
typedef Map::index<uid>::type index_by_uid;

typedef size_t size_type;

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

>>>>>>> 3a2ac5c9acf296d3cc13427c81c5fea560871399
/// A pdal::Schema is a composition of pdal::Dimension instances that form
/// a point cloud.
class PDAL_DLL Schema
{
    friend class RawPtBuf;
public:
/// @name Constructors
    /// construct an instance given the order and dimensions in the dimensions
    /// vector @param dimensions the list of dimensions (and their order) to
    /// use to construct the Schema

/// @name Equality
    /// Equality
    bool operator==(const Schema& other) const;
    /// Inequality
    bool operator!=(const Schema& other) const;

/// @name Dimension manipulation
    /// adds (copies) a Dimension instance to the Schema
    /// @param dim a Dimension that is copied and added to the end of the Schema
//    void appendDimension(Dimension const& dim);

    DimensionPtr getDimension(const std::string& name) const;
    DimensionPtr getDimension(const std::string& name,
        const std::string& nsName) const;
    DimensionPtr getDimension(int pos) const
        { return m_dimPos[(size_t)pos]; }
    DimensionList getDimensions() const
        { return m_dimPos; }
    DimensionList getDimensions(const std::string& nsName) const;

    /// @return  Number of dimensions in the schema.
    size_t size() const
        { return m_dimPos.size(); }

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
        MetadataNode m = MetadataNode());
    
/// @name Private Attributes
private:
    std::unordered_map<std::string, DimensionPtr> m_index;
    std::unordered_map<std::string, DimensionPtr> m_fqIndex;
    std::vector<DimensionPtr> m_dimPos;
    //ABELL - make sure this gets set
    size_t m_byteSize;

    /// @return the total cumulative byte size of all dimensions
    size_t getByteSize() const
        { return m_byteSize; }
};
typedef std::shared_ptr<Schema> SchemaPtr;

PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Schema const& d);

} // namespace pdal

