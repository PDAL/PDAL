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

#include <pdal/pdal.hpp>

#include <vector>
#include <map>

#include <pdal/Dimension.hpp>
#include <pdal/DimensionLayout.hpp>

// boost
#include <boost/cstdint.hpp>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <boost/optional.hpp>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>



namespace pdal
{

/// Schema definition
class PDAL_DLL Schema
{
public:
    Schema();
    Schema(Schema const& other);

    Schema& operator=(Schema const& rhs);

    bool operator==(const Schema& other) const;
    bool operator!=(const Schema& other) const;

    void appendDimension(Dimension const& dim);

    const std::vector<Dimension>& getDimensions() const;
    std::vector<Dimension>& getDimensions();

    bool hasDimension(const DimensionId::Id& id) const;
    //bool hasDimension(const Dimension& dim) const;

    Dimension& getDimension(const DimensionId::Id& id);
    const Dimension& getDimension(const DimensionId::Id& id) const;

    const Dimension& getDimension(std::size_t index) const;
    Dimension& getDimension(std::size_t index);
    
    int getDimensionIndex(const DimensionId::Id& id) const;
    int getDimensionIndex(const Dimension& dim) const;

    void recalculateSizes();

    /// Fetch total byte size -- sum of all dimensions
    inline std::size_t getByteSize() const
    {
        return m_byteSize;
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
    void calculateSizes();

    std::vector<Dimension> m_dimensions;
    std::size_t m_byteSize;

    std::map<DimensionId::Id, std::size_t> m_dimensions_map;
};


PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Schema const& d);

} // namespace liblas

#endif // PDAL_SCHEMA_HPP_INCLUDED
