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

namespace schema {
struct name{};
struct position{};
struct index{};
struct uid{};

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

}

/// Schema definition
class PDAL_DLL Schema
{
public:
    Schema();
    Schema(std::vector<Dimension> const& dimensions);
    Schema(Schema const& other);

    Schema& operator=(Schema const& rhs);

    bool operator==(const Schema& other) const;
    bool operator!=(const Schema& other) const;

    void appendDimension(Dimension const& dim);
    
    schema::Map const& getDimensions() const { return m_index; }
    
    const Dimension& getDimension(std::string const& name, std::string const& ns="") const;
    const Dimension& getDimension(dimension::id const& id) const;
    const Dimension& getDimension(std::size_t index) const;

    boost::optional<Dimension const&> getDimensionOptional(std::string const& name, std::string const& ns="") const;
    boost::optional<Dimension const&> getDimensionOptional(dimension::id const& id) const;
    boost::optional<Dimension const&> getDimensionOptional(std::size_t index) const;

    bool setDimension(Dimension const& );
    
    int getDimensionIndex(const Dimension& dim) const;


    /// Fetch total byte size -- sum of all dimensions
    inline schema::size_type const& getByteSize() const
    {
        return m_byteSize;
    }

    inline std::size_t size() const
    {
        return m_index.get<schema::name>().size();
    }

    // returns a ptree reprsenting the Schema
    //
    // looks like this:
    //    dimension:
    //        [Dimension ptree]
    //    dimension:
    //        [Dimension ptree]
    //    ...
    //
    boost::property_tree::ptree toPTree() const;
   
    void dump() const;

    static Schema from_xml(std::string const& xml, std::string const& xsd);
    static Schema from_xml(std::string const& xml);
    static std::string to_xml(Schema const& schema);

private:
    
    schema::size_type m_byteSize;

    schema::Map m_index;

};


PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Schema const& d);

} // namespace liblas

#endif // PDAL_SCHEMA_HPP_INCLUDED
