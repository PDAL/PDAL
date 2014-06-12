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
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <vector>

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
    ByteArray(std::vector<boost::uint8_t> const& data) : m_bytes(data)
    {}
    
    ByteArray()
    {}

    /** @name Data manipulation
    */
    /// resets the array
    void set(std::vector<boost::uint8_t> const& input)
        { m_bytes = input; }

    /// fetches a reference to the array
    std::vector<boost::uint8_t> const& get() const
        { return m_bytes; }


private:
    std::vector<boost::uint8_t> m_bytes;
};

} //namespace pdal

namespace std
{
extern PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const pdal::ByteArray& output);
extern PDAL_DLL std::istream& operator>>(std::istream& istr,
    pdal::ByteArray& output);
}


namespace pdal
{

class Metadata;
class MetadataNode;
class MetadataNodeImpl;
typedef std::shared_ptr<MetadataNodeImpl> MetadataNodeImplPtr;

class MetadataNodeImpl
{
    friend class MetadataNode;

private:
    MetadataNodeImplPtr add(const std::string& name)
    {
        MetadataNodeImplPtr sub(new MetadataNodeImpl());
        m_subnodes.push_back(sub);
        sub->m_name = name;
        return sub;
    }

    bool operator == (const MetadataNodeImpl& m) const
    {
        if (m_name != m.m_name || m_descrip != m.m_descrip ||
            m_type != m.m_type || m_value != m.m_value)
            return false;
        if (m_subnodes.size() != m.m_subnodes.size())
            return false;

        auto mi2 = m.m_subnodes.begin();
        for (auto mi = m_subnodes.begin(); mi != m_subnodes.end(); ++mi, ++mi2)
        {
            auto mp1 = *mi;
            auto mp2 = *mi2;
            if (!(*mp1 == *mp2))
                return false;
        }
        return true;
    }

    template <typename T>
    inline void setValue(const T& t);
    template <std::size_t N>
    inline void setValue(const char(& c)[N]);

    std::string m_name;
    std::string m_descrip;
    std::string m_type;
    std::string m_value;
    std::vector<MetadataNodeImplPtr> m_subnodes;
};

template <>
inline void MetadataNodeImpl::setValue<bool>(const bool& b)
{
    m_type = "boolean";
    m_value = b ? "true" : "false";
}

template <>
inline void MetadataNodeImpl::setValue<std::string>(const std::string& s)
{
    m_type = "string";
    m_value = s;
}

template <>
inline void MetadataNodeImpl::setValue<const char *>(const char * const & c)
{
    m_type = "string";
    m_value = c;
}

template <std::size_t N>
inline void MetadataNodeImpl::setValue(const char(& c)[N])
{
    m_type = "string";
    m_value = c;
}

template <>
inline void MetadataNodeImpl::setValue<pdal::ByteArray>(
    const pdal::ByteArray& ba)
{
    std::ostringstream oss;
    oss << ba;
    m_type = "base64Binary";
    m_value = oss.str();
}

template <>
inline void MetadataNodeImpl::setValue<float>(const float& f)
{
    m_type = "float";
    m_value = boost::lexical_cast<std::string>(f);
}

template <>
inline void MetadataNodeImpl::setValue<double>(const double& d)
{
    m_type = "double";
    m_value = boost::lexical_cast<std::string>(d);
}

template <>
inline void MetadataNodeImpl::setValue<SpatialReference>(
    const SpatialReference& ref)
{
    std::ostringstream oss;
    oss << ref;
    m_type = "spatialreference";
    m_value = oss.str();
}

template <>
inline void MetadataNodeImpl::setValue<Bounds<double>>(const Bounds<double>& b)
{
    std::ostringstream oss;
    oss << b;
    m_type = "bounds";
    m_value = oss.str();
}

template <>
inline void MetadataNodeImpl::setValue<uint8_t>(const uint8_t& u)
{
    m_type = "nonNegativeInteger";
    m_value = boost::lexical_cast<std::string>((unsigned)u);
}

template <>
inline void MetadataNodeImpl::setValue<uint16_t>(const uint16_t& u)
{
    m_type = "nonNegativeInteger";
    m_value = boost::lexical_cast<std::string>(u);
}

template <>
inline void MetadataNodeImpl::setValue<uint32_t>(const uint32_t& u)
{
    m_type = "nonNegativeInteger";
    m_value = boost::lexical_cast<std::string>(u);
}

template <>
inline void MetadataNodeImpl::setValue<uint64_t>(const uint64_t& u)
{
    m_type = "nonNegativeInteger";
    m_value = boost::lexical_cast<std::string>(u);
}

template <>
inline void MetadataNodeImpl::setValue<int8_t>(const int8_t& i)
{
    m_type = "integer";
    m_value = boost::lexical_cast<std::string>((int)i);
}

template <>
inline void MetadataNodeImpl::setValue<int16_t>(const int16_t& i)
{
    m_type = "integer";
    m_value = boost::lexical_cast<std::string>(i);
}

template <>
inline void MetadataNodeImpl::setValue<int32_t>(const int32_t& i)
{
    m_type = "integer";
    m_value = boost::lexical_cast<std::string>(i);
}

template <>
inline void MetadataNodeImpl::setValue<int64_t>(const int64_t& i)
{
    m_type = "integer";
    m_value = boost::lexical_cast<std::string>(i);
}

template <>
inline void MetadataNodeImpl::setValue<boost::uuids::uuid>(
    const boost::uuids::uuid& u)
{
    std::ostringstream oss;
    oss << u;
    m_type = "uuid";
    m_value = oss.str();
}


class PDAL_DLL MetadataNode
{
    friend class Metadata;
    friend inline
        bool operator == (const MetadataNode& m1, const MetadataNode& m2);
    friend inline
        bool operator != (const MetadataNode& m1, const MetadataNode& m2);

public:
    MetadataNode() : m_impl(new MetadataNodeImpl())
        {}

    MetadataNode add(const std::string& name)
        { return MetadataNode(m_impl->add(name)); }

    template<typename T>
    MetadataNode add(const std::string& name, const T& value,
        const std::string& descrip = std::string())
    {
        MetadataNodeImplPtr impl = m_impl->add(name);
        impl->setValue(value);
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

    std::string type() const
        { return m_impl->m_type; }
    std::string name() const
        { return m_impl->m_name; }
    std::string value() const
        { return m_impl->m_value; }
    std::string description() const
        { return m_impl->m_descrip; }
    std::vector<MetadataNode> children() const
    {
        std::vector<MetadataNode> outnodes;

        const auto& nodes = m_impl->m_subnodes;
        for (auto ci = nodes.begin(); ci != nodes.end(); ++ci)
            outnodes.push_back(MetadataNode(*ci));
        return outnodes;
    }
    bool empty() const
        { return m_impl->m_name.empty() && m_impl->m_value.empty(); }

    template <typename PREDICATE>
    MetadataNode find(PREDICATE p) const
    {
        if (p(*this))
            return *this;
        auto nodes = children();
        for (auto ai = nodes.begin(); ai != nodes.end(); ++ai)
        {
            MetadataNode n = ai->find(p);
            if (!n.empty())
                return n;
        }
        return MetadataNode();
    }

    template <typename PREDICATE>
    MetadataNode findChild(PREDICATE p) const
    {
        auto nodes = children();
        for (auto ai = nodes.begin(); ai != nodes.end(); ++ai)
        {
            MetadataNode& n = *ai;
            if (p(n))
                return n;
        }
        return MetadataNode();
    }

private:
    MetadataNodeImplPtr m_impl;

    MetadataNode(MetadataNodeImplPtr node) : m_impl(node)
        {}
};

inline bool operator == (const MetadataNode& m1, const MetadataNode& m2)
{
    return m1.m_impl == m2.m_impl;
}

inline bool operator != (const MetadataNode& m1, const MetadataNode& m2)
{
    return !(m1.m_impl == m2.m_impl);
}


class Metadata
{
public:
    Metadata()
    {}
   
    Metadata(const std::string& name) : m_name(name)
    {}

    MetadataNode getNode() const
        { return m_root; }

private:
    MetadataNode m_root;
    std::string m_name;
};
typedef std::shared_ptr<Metadata> MetadataPtr;

} // namespace pdal


