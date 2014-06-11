/******************************************************************************
* Copyright (c) 2012, Howard Butler hobu.inc@gmail.com
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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>


#include <boost/shared_array.hpp>
#include <boost/blank.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <boost/optional.hpp>

#include <boost/algorithm/string.hpp>


#include <vector>
#include <map>

namespace pdal
{

/// ByteArray simply wrapps a std::vector<boost::uint8_t> such that it can then
/// be dumped to an ostream in a base64 encoding. For now, it makes a copy of
/// the data it is given, and should not be used for slinging big data around.
class PDAL_DLL ByteArray
{
public:

    /** @name Constructors
    */
    /// Constructs a ByteArray instance with the given array of data.
    ByteArray(std::vector<boost::uint8_t> const& data)
        : m_bytes(data)
    {}
    
    ByteArray()
    {}

    /** @name Data manipulation
    */
    /// resets the array
    inline void set(std::vector<boost::uint8_t> const& input)
    {
        m_bytes = input;
    }

    /// fetches a reference to the array
    inline std::vector<boost::uint8_t> const& get() const
    {
        return m_bytes;
    }


private:
    std::vector<boost::uint8_t> m_bytes;
};


class Metadata;

/// pdal::Metadata is a container for metadata entries that pdal::Stage and
/// pdal::PointBuffer carry around as part of their internal operations. Bits of
/// information might come from a pdal::Reader that opens a file, or a
/// pdal::Filter that processes as an intermediate stage, and
/// pdal::Metadata is what is used to hold and pass those metadata
/// around.  
class PDAL_DLL MetadataNode
{
    friend class Metadata;
private:
    MetadataNode(boost::property_tree::ptree *tree) :
        m_tree(tree)
    {}

public:
    /// Add a stage namespace to a root tree.
    MetadataNode addCategory(const std::string& path)
    {
        boost::property_tree::ptree& pt = m_tree->put(path, "");    
        return MetadataNode(&pt);
    }

    // Get a node representing a category.
    MetadataNode getCategory(std::string const& path)
    {
        auto it = m_tree->find(path);
        if (it == m_tree->not_found())
        {
            std::ostringstream oss;
            oss << "Node with path '" << path << "' not found";
            throw metadata_not_found(oss.str());
        }
        return MetadataNode(&(it->second));
    } 

    // Add a standard set of metadata to a node.
    template <typename T>
    void add(const std::string& name, const T& value,
        const std::string& description = std::string())
    {
        setName(name);
        if (description.size())
            setDescription(description);
        setValue(value);
    }
    
    // Get the metadata node type.
    std::string getType() const
    {
        return m_tree->get<std::string>("type");
    }

    /// sets the type for the metadata entry as a string.  These 
    /// types are roughly mapped to the XSD typenames like integer, 
    /// nonNegativeInteger, etc
    /// @param t value for the type of the entry.  
    void setType(std::string const& t)
    {
        m_tree->put<std::string>("type", t);
    }

    /** @name value
    */
    /*! 
        \param v value to set for this entry
        \verbatim embed:rst
        .. note::

            The type T must have a std::ostream<< method and be 
            copy-constructable to meet the requirements if i/o'ing through a 
            boost::property_tree.
        \endverbatim
    */
    template <class T> inline void setValue(T const& v);
    template <class T> T getValue() const
        { return m_tree->get<T>("value"); } 

    /*! \return the value at the given sub-path.
        \param path The single-name sub-path to select the value
        \verbatim embed:rst
        .. note::

            The path given here is the sub-path inside of the `metadata` node 
            of the boost::property_tree::ptree.  It is not a full node path.
        \endverbatim
    */
    /**
    template <class T> inline T getValue(std::string const& path) const 
    { 
        return m_tree.get<T>("metadata."+ path +".value"); 
    } 
    **/
    
    /*! \return a boost::optional-wrapped instance for the given sub-path.
        \param path The single-name sub-path to select the value
        \verbatim embed:rst
        .. note::

            The path given here is the sub-path inside of the `metadata` node 
            of the boost::property_tree::ptree.  It is not a full node path.
        \endverbatim
    template <class T> boost::optional<T>
    {
        return m_tree.get_optional<T>( path ".value"); 
    } 

    */

    template <typename T>
    T getValue(const std::string& path, T& def) const
    {
        return m_tree->get<T>(path, def);
    }

    
    bool deleteMetadata(std::string const& path)
    {
        return m_tree->erase(path);
    }

    /** @name name
    */
    /// returns the name for the metadata entry
    inline std::string getName() const
        { return m_tree->get<std::string>("name"); }

    /// resets the name for the metadata entry
    /// @param name value to use for new name
    void setName(std::string const& name)
        { m_tree->put("name", name); }

    /** @name description
    */
    /// sets the description for the Metadata entry
    /// @param description new value to use for the description of the Metadata
    void setDescription(const std::string& description)
        { m_tree->put("description", description); }

    /// @return the description of the Metadata entry
    std::string getDescription() const
        { return m_tree->get<std::string>("description"); }

    /// @name boost::property_tree::ptree output
    /// @return the pdal::Metadata instance as a boost::property_tree::ptree
    boost::property_tree::ptree const& toPTree() const 
        { return *m_tree; } 

    bool operator != (const MetadataNode& other)
        { return *m_tree != *(other.m_tree); } 

    bool operator == (const MetadataNode& other)
        { return *m_tree == *(other.m_tree); } 

private:
    boost::property_tree::ptree *m_tree;
};

template <>
inline void MetadataNode::setValue<bool>(bool const& v)
{
    setType("boolean");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<std::string>(std::string const& v)
{
    setType("string");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<pdal::ByteArray>(pdal::ByteArray const& v)
{
    setType("base64Binary");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<float>(float const& v)
{
    setType("float");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<double>(double const& v)
{
    setType("double");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<pdal::SpatialReference>(
    pdal::SpatialReference const& v)
{
    setType("spatialreference");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<pdal::Bounds<double>>(pdal::Bounds<double> const& v)
{
    setType("bounds");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::uint8_t>(boost::uint8_t const& v)
{
    setType("nonNegativeInteger");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::uint16_t>(boost::uint16_t const& v)
{
    setType("nonNegativeInteger");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::uint32_t>(boost::uint32_t const& v)
{
    setType("nonNegativeInteger");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::uint64_t>(boost::uint64_t const& v)
{
    setType("nonNegativeInteger");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::int8_t>(boost::int8_t const& v)
{
    setType("integer");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::int16_t>(boost::int16_t const& v)
{
    setType("integer");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::int32_t>(boost::int32_t const& v)
{
    setType("integer");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<int64_t>(int64_t const& v)
{
    setType("integer");
    m_tree->put("value", v);
}

template <>
inline void MetadataNode::setValue<boost::uuids::uuid>(
    boost::uuids::uuid const& v)
{
    setType("uuid");
    m_tree->put("value",v);
}

template <>
inline void MetadataNode::setValue<MetadataNode>(MetadataNode const& v)
{
    setType("metadata");
    m_tree->add_child("value", *(v.m_tree));
}

/**
void Metadata::add(Metadata const& m)
{
    std::string n = boost::algorithm::ireplace_all_copy(m.getName(), ".", "_");
    m_tree.add_child("metadata."+n, m.toPTree());
}

inline void Metadata::setMetadata(Metadata const& m)
{
    std::string n = boost::algorithm::ireplace_all_copy(m.getName(), ".", "_");    
    deleteMetadata(n);
    addMetadata(m);
}

template <typename T>
inline void Metadata::addMetadata(std::string const& name, T const& value, 
    std::string const& description)
{
    Metadata m(name, value, description);
    addMetadata(m);
}
**/


class Metadata
{
public:
    Metadata() : m_baseTree(m_tree.put("metadata", ""))
    {}

    //ABELL - This should go away with the buffer's metadata.
    Metadata(const Metadata& m) : m_tree(m.m_tree),
        m_baseTree(m_tree.get_child("metadata"))
    {}

    //ABELL - This should go away with the buffer's metadata.
    Metadata& operator = (const Metadata& m)
    {
        m_tree = m.m_tree;
        m_baseTree = m_tree.get_child("metadata");
        return *this;
    }


    MetadataNode getNode()
        { return MetadataNode(&m_baseTree); }

private:
    boost::property_tree::ptree m_tree;
    boost::property_tree::ptree& m_baseTree;
};
typedef std::shared_ptr<Metadata> MetadataPtr;

} // namespace pdal


namespace std
{
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const pdal::ByteArray& output);
extern PDAL_DLL std::istream& operator>>(std::istream& istr,
    pdal::ByteArray& output);
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const pdal::Metadata& m);
}

