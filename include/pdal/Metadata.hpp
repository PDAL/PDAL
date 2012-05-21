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
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/functional/hash.hpp>



#include <vector>
#include <map>

namespace pdal
{

// Forward decl for boost::recursive_variant
class Metadata;

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

namespace metadata
{

typedef boost::uuids::uuid id;
typedef boost::shared_ptr<Metadata> MetadataPtr;

enum Type
{
    /// boolean
    Boolean,
    /// equivalent to int32_t
    SignedInteger,
    /// equivalent to uint32_t
    UnsignedInteger,
    /// equivalent to float
    Float,
    /// equivalent to double
    Double,
    /// equivalent to std::string
    String,
    /// raw binary data in the form std::vector<boost::uint8_t>
    Bytes,
    /// A pdal::Bounds instance
    Bounds,
    /// A pdal::SpatialReference instance
    SpatialReference,
    /// A boost::uuids::uuid instance
    UUID,
    /// A pdal::Metadata instance
    MData,
    /// Nothing
    Blank
};


typedef boost::variant< bool,
                        boost::blank,
                        float,
                        double,
                        boost::int8_t,
                        boost::uint8_t,
                        boost::int16_t,
                        boost::uint16_t,
                        boost::int32_t,
                        boost::uint32_t,
                        boost::int64_t,
                        boost::uint64_t,
                        boost::uuids::uuid,
                        std::string,
                        pdal::ByteArray,
                        pdal::SpatialReference,
                        pdal::Bounds<double>,
                        boost::recursive_wrapper<Metadata> > Variant;




/// metadata::Entry is a container for metadata entries that pdal::Stage and
/// pdal::PointBuffer carry around as part of their internal operations. Bits of
/// information might come from a pdal::Reader that opens a file, or a
/// pdal::Filter that processes as an intermediate stage, and
/// pdal::metadata::Entry is what is used to hold and pass those metadata
/// around.  pdal::metadata::Entry values must be of type
/// pdal::metadata::Variant and it is required that they are serializeable to
/// std::string.

/// pdal::metadata::Entry instances also carry with them a map of key/value
/// pairs called attributes that are metadata about the metadata entry. For
/// example, a LAS VLR might have a name of "classification", a pdal::ByteArray
/// for its data, and a set of attributes that are "userid":"4321" and
/// "vlrid":"1234". These other metadata may be useful given the context, and
/// any auxiliary data about the metadata entry should be provided via
/// attributes. It is up to you to determine where the line of attribute and new
/// metadata entry exists.
class PDAL_DLL Entry : public boost::property_tree::ptree
{
public:

    /** @name Constructors
    */
    /// Base constructor
    /// @param name entry name to use for this metadata entry
    Entry(std::string const& name);

    template <typename T>
    Entry(std::string const& name, const T& value, std::string const& description="")
    {
        setName(name);
        setDescription(description);
        put_value(value);
    }


    /** @name entry type
    */
    /// returns the pdal::metadata::Type for the metadata entry
    inline metadata::Type getType() const
    {
        return static_cast<metadata::Type>(boost::property_tree::ptree::get<boost::uint32_t>("type"));
    }

    /// sets the pdal::metadata::Type for the metadata entry
    /// @param t pdal::metadata::Type value for the entry
    inline void setType(metadata::Type t)
    {
        boost::property_tree::ptree::put("type", t);
    }

    /// returns a std::string representation of the type
    std::string getTypeName() const;

    /** @name entry value
    */
    /// @param v value of type pdal::metadata::Variant to set for the entry
    template <class T> inline void put_value(T const& v);

    /** @name entry name
    */
    /// returns the name for the metadata entry
    inline std::string getName() const
    {
        return boost::property_tree::ptree::get<std::string>("name");
    }

    /// resets the name for the metadata entry
    /// @param name value to use for new name
    inline void setName(std::string const& name)
    {
        boost::property_tree::ptree::put("name", name);
    }


/// @name Serialization
    boost::property_tree::ptree toPTree() const;

    // Create a single entry, non-valid xml string to represent the Entry
    std::string to_xml();

    /** @name description
    */
    /// Overwrites the description given in the constructor
    /// @param description new value to use for the description of the Option
    inline void setDescription(const std::string& description)
    {
        boost::property_tree::ptree::put("description", description);
    }

    /// @return the description of the Option
    inline std::string getDescription() const
    {
        return  boost::property_tree::ptree::get<std::string>("description");;
    }

};


extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const metadata::Entry& srs);

template <>
inline void metadata::Entry::put_value<bool>(bool const& v)
{
    setType(metadata::Boolean);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<std::string>(std::string const& v)
{
    setType(metadata::String);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<pdal::ByteArray>(pdal::ByteArray const& v)
{
    setType(metadata::Bytes);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<float>(float const& v)
{
    setType(metadata::Float);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<double>(double const& v)
{
    setType(metadata::Double);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<pdal::SpatialReference>(pdal::SpatialReference const& v)
{
    setType(metadata::SpatialReference);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<pdal::Bounds<double> >(pdal::Bounds<double> const& v)
{
    setType(metadata::Bounds);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::uint8_t>(boost::uint8_t const& v)
{
    setType(metadata::UnsignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::uint16_t>(boost::uint16_t const& v)
{
    setType(metadata::UnsignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::uint32_t>(boost::uint32_t const& v)
{
    setType(metadata::UnsignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::uint64_t>(boost::uint64_t const& v)
{
    setType(metadata::UnsignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::int8_t>(boost::int8_t const& v)
{
    setType(metadata::SignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::int16_t>(boost::int16_t const& v)
{
    setType(metadata::SignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::int32_t>(boost::int32_t const& v)
{
    setType(metadata::SignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::int64_t>(boost::int64_t const& v)
{
    setType(metadata::SignedInteger);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::uuids::uuid>(boost::uuids::uuid const& v)
{
    setType(metadata::UUID);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<pdal::Metadata>(pdal::Metadata const& v)
{
    setType(metadata::MData);
    boost::property_tree::ptree::put_value(v);
}

template <>
inline void metadata::Entry::put_value<boost::blank>(boost::blank const& v)
{
    setType(metadata::Blank);
    boost::property_tree::ptree::put_value(v);
}




struct name {};
struct index {};
// struct uid{};
//
typedef boost::multi_index::multi_index_container<
metadata::Entry,
         boost::multi_index::indexed_by<

         boost::multi_index::random_access<boost::multi_index::tag<index> >,
         // sort by less<string> on GetName
         boost::multi_index::hashed_unique<boost::multi_index::tag<name>, boost::multi_index::const_mem_fun<metadata::Entry,std::string,&metadata::Entry::getName> >
         // boost::multi_index::hashed_non_unique<boost::multi_index::tag<uid>, boost::multi_index::const_mem_fun<metadata::Entry,metadata::id const&,&metadata::Entry::getUUID> >
         >
         > EntryMap;

typedef EntryMap::index<name>::type index_by_name;
typedef EntryMap::index<index>::type index_by_index;
// typedef EntryMap::index<uid>::type index_by_uid;


}

/// Metadata is a container for Metadata entries.
class PDAL_DLL Metadata
{
public:

    /** @name Constructors
    */

    Metadata() {};

    /// Copy constructor
    Metadata(const Metadata&);
    Metadata& operator=(const Metadata&);

    ~Metadata()
    {
        return;
    }

    /** @name Operators
    */
    /// Addition operator
    Metadata operator+(const Metadata& rhs) const;

    /** @name entry type
    */

    /// add a Metadata entry to the PointBuffer's metadata map
    void addEntry(pdal::metadata::Entry const& entry);

    /*! add a new value T metadata for the given metadata::Entry key.
        \param name metadata::Entry entry key to use
        \param value the T value to set.
    */
    template<class T> void addEntry(std::string const& name, T value);


    /// @return a const& to a metadata::Entry entry with the given name.
    /// If none is found, pdal::metadata_not_found is thrown.
    /// @param name name to use when searching
    /// @param ns to use when searching for metadata entry
    metadata::Entry const& getEntry(std::string const& name) const;

    // /// @return a const& to metadata::Entry entry with given metadata::id.
    // /// If none is found, pdal::metadata_not_found is thrown.
    // /// @param v metadata::id to search for.
    // metadata::Entry const& getMetadata(metadata::id const& v) const;

    /// @return a const& to metadata::Entry with the given index. If the
    /// index is out of range, pdal::metadata_not_found is thrown.
    /// @param index position index to return.
    metadata::Entry const& getEntry(std::size_t index) const;

    /// @return the number of metadata::Entry entries in the map
    inline metadata::EntryMap::size_type size() const
    {
        return m_metadata.get<metadata::index>().size();
    }

    /// @return a EntryMap copy to use for setting the metadata::Entry on another
    /// PointBuffer with setMetadata()
    /// @param index position index to return.
    inline metadata::EntryMap const& getMetadata() const
    {
        return m_metadata;
    }

    /// @return a boost::optional-wrapped const& to a metadata::Entry with the given name
    /// If no matching metadata entry is found, the optional will be empty.
    /// @param name name to use when searching
    /// matching metadata::Entry instance with name \b name is returned.
    boost::optional<metadata::Entry const&> getEntryOptional(std::string const& name) const;


    /// @return a boost::optional-wrapped const& to a metadata::Entry with the given
    /// index. If the index is out of range, the optional will be empty.
    /// @param index position index to return.
    boost::optional<metadata::Entry const&> getEntryOptional(std::size_t index) const;

    /*! overwrites an existing metadata::Entry with the same name as m
        \param m the metadata::Entry instance that contains the name
        to overwrite in the PointBuffer.
        \verbatim embed:rst
        .. warning::

            If no namespace is given, the *first* metadata entry with a matching
            :cpp:func:`pdal::metadata::Entry::getName()` will be overwritten.

        \endverbatim
    */
    bool setEntry(metadata::Entry const& m);

    /*! reset the value T metadata for the given metadata::Entry key and namespace.
        \param name metadata::Entry entry key to use
        \param value the T value to set.
    */
    template<class T> void setEntry(std::string const& name, T value);


    /// sets the EntryMap for the PointBuffer
    /// @param v EntryMap instance to use (typically from another PointBuffer)
    void setMetadata(metadata::EntryMap const& v)
    {
        m_metadata = v;
    }

    /// @name Serialization
    boost::property_tree::ptree toPTree() const;

    /// XML output
    std::string to_xml() const;

    /// JSON output
    std::string to_json() const;


    /** @name private attributes
    */
private:

    metadata::EntryMap m_metadata;

};

template <class T>
inline void Metadata::addEntry(std::string const& name, T value)
{
    metadata::Entry m(name);
    try
    {
        m.put_value<T>(value);
    } catch (...)
    {
        m.put_value<boost::blank>(boost::blank());
    }
    addEntry(m);
    return;
}

template <class T>
inline void Metadata::setEntry(std::string const& name, T value)
{
    metadata::Entry m(name);
    try
    {
        m.put_value<T>(value);
    } catch (...)
    {
        m.put_value<boost::blank>(boost::blank());
    }
    setEntry(m);

    return;
}


} // namespace pdal

namespace std
{

///
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::ByteArray& output);
extern PDAL_DLL std::istream& operator>>(std::istream& istr, pdal::ByteArray& output);

extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const pdal::Metadata& metadata);
}

#endif
