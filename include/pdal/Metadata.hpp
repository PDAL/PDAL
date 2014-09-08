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

#include <map>
#include <vector>

namespace pdal
{

namespace MetadataType
{
enum Enum
{
    Instance,
    Array
};
}

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
typedef std::vector<MetadataNodeImplPtr> MetadataImplList;
typedef std::map<std::string, MetadataImplList> MetadataSubnodes;

class MetadataNodeImpl
{
    friend class MetadataNode;

private:
    MetadataNodeImpl(const std::string& name) :
        m_name(name), m_kind(MetadataType::Instance)
    {}

    MetadataNodeImpl() : m_kind(MetadataType::Instance)
    {}

    void makeArray(MetadataImplList& l)
    {
        for (auto li = l.begin(); li != l.end(); ++li)
        {
            MetadataNodeImplPtr node = *li;
            node->m_kind = MetadataType::Array;
        }
    }

    MetadataNodeImplPtr add(const std::string& name)
    {
        MetadataNodeImplPtr sub(new MetadataNodeImpl(name));
        MetadataImplList& l = m_subnodes[name];
        l.push_back(sub);
        if (l.size() > 1)
            makeArray(l);
        return sub;
    }

    MetadataNodeImplPtr addList(const std::string& name)
    {
        MetadataNodeImplPtr sub(new MetadataNodeImpl(name));
        MetadataImplList& l = m_subnodes[name];
        l.push_back(sub);
        makeArray(l);
        return sub;
    }

    MetadataNodeImplPtr add(MetadataNodeImplPtr node)
    {
        MetadataImplList& l = m_subnodes[node->m_name];
        l.push_back(node);
        if (l.size() > 1)
            makeArray(l);
        return node;
    }

    MetadataNodeImplPtr addList(MetadataNodeImplPtr node)
    {
        MetadataImplList& l = m_subnodes[node->m_name];
        l.push_back(node);
        makeArray(l);
        return node;
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
            if (mi->first != mi2->first)
                return false;
            const MetadataImplList& ml = mi->second;
            const MetadataImplList& ml2 = mi->second;
            if (ml.size() != ml2.size())
                return false;
            auto li2 = ml2.begin();
            for (auto li = ml.begin(); li != ml.end(); ++li, ++li2)
            {
                auto node1 = *li;
                auto node2 = *li2;
                if (!(*node1 == *node2))
                    return false;
            }
        }
        return true;
    }

    template <typename T>
    inline void setValue(const T& t);

    template <std::size_t N>
    inline void setValue(const char(& c)[N]);

    MetadataImplList& subnodes(const std::string &name)
    {
        auto si = m_subnodes.find(name);
        if (si != m_subnodes.end())
            return si->second;

        static MetadataImplList l;
        return l;
    }

    const MetadataImplList& subnodes(const std::string& name) const
    {
        MetadataNodeImpl *nc_this = const_cast<MetadataNodeImpl *>(this);
        return nc_this->subnodes(name);
    }

    MetadataType::Enum nodeType(const std::string& name) const
    {
        const MetadataImplList& l = subnodes(name);
        if (l.size())
        {
            MetadataNodeImplPtr node = *l.begin();
            return node->m_kind;
        }
        return MetadataType::Instance;
    }


    std::string toJSON() const;
    void toJSON(std::ostream& o, int level) const;
    void subnodesToJSON(std::ostream& o, int level) const;

    std::string m_name;
    std::string m_descrip;
    std::string m_type;
    std::string m_value;
    MetadataType::Enum m_kind;
    MetadataSubnodes m_subnodes;
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

    MetadataNode(const std::string& name) : m_impl(new MetadataNodeImpl(name))
        {}

    MetadataNode add(const std::string& name)
        { return MetadataNode(m_impl->add(name)); }

    MetadataNode addList(const std::string& name)
        { return MetadataNode(m_impl->addList(name)); }

    MetadataNode add(MetadataNode node)
        { return MetadataNode(m_impl->add(node.m_impl)); }

    MetadataNode addList(MetadataNode node)
        { return MetadataNode(m_impl->addList(node.m_impl)); }

    template<typename T>
    MetadataNode add(const std::string& name, const T& value,
        const std::string& descrip = std::string())
    {
        MetadataNodeImplPtr impl = m_impl->add(name);
        impl->setValue(value);
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

    template<typename T>
    MetadataNode addList(const std::string& name, const T& value,
        const std::string& descrip = std::string())
    {
        MetadataNodeImplPtr impl = m_impl->addList(name);
        impl->setValue(value);
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

    template<typename T>
    MetadataNode addOrUpdate(const std::string& lname, const T& value)
    {
        if (m_impl->nodeType(lname) == MetadataType::Array)
            throw pdal_error("Can't call addOrUpdate() on subnode list.");
        MetadataImplList& l = m_impl->subnodes(lname);

        if (l.empty())
            return add(lname, value);
        MetadataNodeImplPtr impl = *l.begin();
        impl->setValue(value);
        return MetadataNode(impl);
    }

    template<typename T>
    MetadataNode addOrUpdate(const std::string& lname, const T& value,
        const std::string& descrip)
    {
        MetadataNode m = addOrUpdate(lname, value);
        m_impl->m_descrip = descrip;
        return m;
    }

    std::string type() const
        { return m_impl->m_type; }
    std::string kind() const
        {
            if (m_impl->m_kind == MetadataType::Array)
                return "array";
            else
                return "instance";
        }
    std::string name() const
        { return m_impl->m_name; }
    std::string value() const
        { return m_impl->m_value; }
    std::string description() const
        { return m_impl->m_descrip; }
    std::vector<MetadataNode> children() const
    {
        std::vector<MetadataNode> outnodes;

        const MetadataSubnodes& nodes = m_impl->m_subnodes;
        for (auto si = nodes.begin(); si != nodes.end(); ++si)
        {
            const MetadataImplList& l = si->second;
            for (auto li = l.begin(); li != l.end(); ++li)
                outnodes.push_back(MetadataNode(*li));
        }
        return outnodes;
    }
    std::vector<MetadataNode> children(const std::string& name) const
    {
        std::vector<MetadataNode> outnodes;

        auto si = m_impl->m_subnodes.find(name);
        if (si != m_impl->m_subnodes.end())
        {
            const MetadataImplList& l = si->second;
            for (auto li = l.begin(); li != l.end(); ++li)
                outnodes.push_back(MetadataNode(*li));
        }
        return outnodes;
    }
    bool operator ! ()
        { return empty(); }
    bool valid() const
        { return !empty(); }
    bool empty() const
        { return m_impl->m_name.empty(); }

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

    MetadataNode findChild(const char *s) const
        { return findChild(std::string(s)); }

    MetadataNode findChild(std::string s) const
    {
        auto splitString = [](std::string& s) -> std::string
        {
            std::string val;
            size_t pos = s.find(':');
            if (pos == std::string::npos)
            {
                val = s;
                s.clear();
            }
            else
            {
                val = s.substr(0, pos);
                s = (pos == s.size() - 1) ? "" : s.substr(pos + 1);
            }
            return val;
        };

        if (s.empty())
            return *this;
        std::string lname = splitString(s);
        auto nodes = children(lname);
        for (auto ai = nodes.begin(); ai != nodes.end(); ++ai)
        {
            MetadataNode& n = *ai;
            MetadataNode child = n.findChild(s);
            if (!child.empty())
                return child;
        }
        return MetadataNode();
    }

    std::string toJSON() const;

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
    friend class PointContext;

public:
    Metadata() : m_root("root"), m_private("private")
    {}

    Metadata(const std::string& name) : m_name(name)
    {}

    MetadataNode getNode() const
        { return m_root; }

private:
    MetadataNode m_root;
    MetadataNode m_private;
    std::string m_name;

    MetadataNode privateNode() const
        { return m_private; }
};
typedef std::shared_ptr<Metadata> MetadataPtr;

} // namespace pdal


