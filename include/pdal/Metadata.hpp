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

#ifndef INCLUDED_METADATA_HPP
#define INCLUDED_METADATA_HPP

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>


#include <boost/shared_array.hpp>
#include <boost/variant.hpp>
#include <boost/variant/recursive_variant.hpp>
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
/// be dumped to an ostream in a base64 encoding. For now, it makes a copy of the data
/// it is given, and should not be used for slinging big data around.
class PDAL_DLL ByteArray
{
public:

    /** @name Constructors
    */
    /// Constructs a ByteArray instance with the given array of data.
    ByteArray(std::vector<boost::uint8_t> const& data)
        : m_bytes(data)
    {
    }
    
    ByteArray()
    {
    }

    /// Copy constructor
    ByteArray(const ByteArray& rhs)
        : m_bytes(rhs.m_bytes)

    {
    }

    /// Assignment operator
    inline ByteArray& operator=(ByteArray const& rhs)
    {
        if (&rhs != this)
        {
            m_bytes = rhs.m_bytes;
        }
        return *this;
    }

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


    /** @name private attributes
    */
private:

    std::vector<boost::uint8_t> m_bytes;
};



/// metadata::Metadata is a container for metadata entries that pdal::Stage and
/// pdal::PointBuffer carry around as part of their internal operations. Bits of
/// information might come from a pdal::Reader that opens a file, or a
/// pdal::Filter that processes as an intermediate stage, and
/// pdal::Metadata is what is used to hold and pass those metadata
/// around.  
class PDAL_DLL Metadata
{
public:

    /** @name Constructors
    */
    /// Base constructor
    /// The Metadata::getType will be "blank" and the returned Metadata::getValue 
    /// will be `0` instances made with this constructor
    Metadata();

    /// Defined name constructor
    /// @param name entry name to use for this metadata entry
    /// The Metadata::getType will be "blank" and the returned Metadata::getValue 
    /// will be `0` instances made with this constructor
    Metadata(std::string const& name);
    
    /// Copy constructor
    Metadata(Metadata const& other);

    /// Property constructor. The boost::property_tree::ptree should be in the 
    /// same form and have the same nodes as those fetched from a Metadata::toPTree() 
    /// call.
    Metadata(boost::property_tree::ptree const& tree);
    
    /*! 
        Convenience constructor
        
        Equivalent to manually calling Metadata::setName,
        Metadata::setDescription, and Metadata::setValue individually for the
        instance.
        \param name the name to use for this instance
        \param value the value to set for this instance.
        \param description the description to use for this Metadata entry.
        \verbatim embed:rst
        .. note::

            This method has a side effect of calling setType with an appropriate 
            value as determined by the type T.
        \endverbatim
    */
    template <typename T>
    Metadata(std::string const& name, const T& value, std::string const& description="")
    {
        setName(name);
        setDescription(description);
        setValue<T>(value);
    }


    /// @name Adding Metadata members
    /// Convenience addition
    /// @param name the name to use for the new Metadata instance to be added to this instance.
    /// @param value the value to set for the new Metadata instance to be added to this instance.
    /// @param description the description to set for the new Metadata instance to be added to this instance.
    template <typename T>
    void addMetadata(   std::string const& name, 
                        const T& value, 
                        std::string const& description="");
    
    /// Add a new Metadata instance to this instance.
    void addMetadata( Metadata const& m);


    
    /// @name Operators
    
    /// @return a new Metadata instance that is the sum of
    /// two Metadata instances together by placing each in their 
    /// respective paths as defined by their dotted Metadata::getName() values
    Metadata operator+(const Metadata& rhs) const;
    
    /** @name entry type
    */
    /// @return the type for the metadata entry as a string.  These 
    /// types are roughly mapped to the XSD typenames like integer, 
    /// nonNegativeInteger, etc
    inline std::string getType() const
    {
        return m_tree.get<std::string>("type");
    }

    /// sets the type for the metadata entry as a string.  These 
    /// types are roughly mapped to the XSD typenames like integer, 
    /// nonNegativeInteger, etc
    /// @param t value for the type of the entry.  
    inline void setType(std::string const& t)
    {
        m_tree.put<std::string>("type", t);
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
    template <class T> inline T getValue() const { return m_tree.get<T>("value"); } 

    /*! \return the value at the given sub-path.
        \param path The single-name sub-path to select the value
        \verbatim embed:rst
        .. note::

            The path given here is the sub-path inside of the `metadata` node 
            of the boost::property_tree::ptree.  It is not a full node path.
        \endverbatim
    */
    template <class T> inline T getValue(std::string const& path) const 
    { 
        return m_tree.get<T>("metadata."+path+".value"); 
    } 
    
    /*! \return a boost::optional-wrapped instance for the given sub-path.
        \param path The single-name sub-path to select the value
        \verbatim embed:rst
        .. note::

            The path given here is the sub-path inside of the `metadata` node 
            of the boost::property_tree::ptree.  It is not a full node path.
        \endverbatim
    */
    template <class T> inline boost::optional<T> getValueOptional(std::string const& path) const 
    { 
        return m_tree.get_optional<T>("metadata." + path+".value"); 
    } 

    /// @return a Metadata instance at a given path (in 
    /// boost::property_tree::ptree parlance).
    inline Metadata getMetadata(std::string const& path) const 
    { 
        return Metadata(m_tree.get_child(path)); 
    } 


    /** @name name
    */
    /// returns the name for the metadata entry
    inline std::string getName() const
    {
        return m_tree.get<std::string>("name");
    }

    /// resets the name for the metadata entry
    /// @param name value to use for new name
    inline void setName(std::string const& name)
    {
        m_tree.put("name", name);
    }

    /** @name description
    */
    /// sets the description for the Metadata entry
    /// @param description new value to use for the description of the Metadata
    inline void setDescription(const std::string& description)
    {
        m_tree.put("description", description);
    }

    /// @return the description of the Metadata entry
    inline std::string getDescription() const
    {
        return m_tree.get<std::string>("description");
    }

    /// @name boost::property_tree::ptree output
    /// @return the pdal::Metadata instance as a boost::property_tree::ptree
    inline boost::property_tree::ptree const& toPTree() const 
    {
        return m_tree; 
    } 

private:
    boost::property_tree::ptree m_tree;
    
};


template <>
inline void Metadata::setValue<bool>(bool const& v)
{
    setType("boolean");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<std::string>(std::string const& v)
{
    setType("string");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::ByteArray>(pdal::ByteArray const& v)
{
    setType("base64Binary");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<float>(float const& v)
{
    setType("float");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<double>(double const& v)
{
    setType("double");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::SpatialReference>(pdal::SpatialReference const& v)
{
    setType("spatialreference");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::Bounds<double> >(pdal::Bounds<double> const& v)
{
    setType("bounds");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint8_t>(boost::uint8_t const& v)
{
    setType("nonNegativeInteger");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint16_t>(boost::uint16_t const& v)
{
    setType("nonNegativeInteger");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint32_t>(boost::uint32_t const& v)
{
    setType("nonNegativeInteger");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uint64_t>(boost::uint64_t const& v)
{
    setType("nonNegativeInteger");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int8_t>(boost::int8_t const& v)
{
    setType("integer");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int16_t>(boost::int16_t const& v)
{
    setType("integer");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int32_t>(boost::int32_t const& v)
{
    setType("integer");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::int64_t>(boost::int64_t const& v)
{
    setType("integer");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<boost::uuids::uuid>(boost::uuids::uuid const& v)
{
    setType("uuid");
    m_tree.put("value",v);
}

template <>
inline void Metadata::setValue<pdal::Metadata>(pdal::Metadata const& v)
{
    setType("metadata");
    m_tree.add_child("value",v.toPTree());
}

template <>
inline void Metadata::setValue<boost::blank>(boost::blank const& v)
{
    setType("blank");
    m_tree.put("value",v);
}

inline void Metadata::addMetadata(  Metadata const& m)
{
    
    std::string n = boost::algorithm::ireplace_all_copy(m.getName(), ".", "_");
    m_tree.add_child("metadata."+n, m.toPTree());
}

template <typename T>
inline void Metadata::addMetadata(  std::string const& name, 
                                    T const& value, 
                                    std::string const& description)
{
    Metadata m(name, value, description);
    addMetadata(m);
}


} // namespace pdal

namespace std
{

///
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& output);
extern PDAL_DLL std::istream& operator>>(std::istream& istr, pdal::ByteArray& output);
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& m);


// extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& metadata);
}

#endif
