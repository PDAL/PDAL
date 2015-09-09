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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <pdal/util/Bounds.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/SpatialReference.hpp>

#include <map>
#include <memory>
#include <vector>
#include <stdint.h>

namespace pdal
{

namespace MetadataType
{
enum Enum
{
    Instance,
    Array
};
} // namespace MetadataType

class Metadata;
class MetadataNode;
class MetadataNodeImpl;
typedef std::shared_ptr<MetadataNodeImpl> MetadataNodeImplPtr;
typedef std::vector<MetadataNodeImplPtr> MetadataImplList;
typedef std::map<std::string, MetadataImplList> MetadataSubnodes;
typedef std::vector<MetadataNode> MetadataNodeList;

class MetadataNodeImpl
{
    friend class MetadataNode;

private:
    MetadataNodeImpl(const std::string& name) : m_kind(MetadataType::Instance)
        { m_name = name; }

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

    std::string m_name;
    std::string m_descrip;
    std::string m_type;
    std::string m_value;
    MetadataType::Enum m_kind;
    MetadataSubnodes m_subnodes;
};

template <>
inline void MetadataNodeImpl::setValue(const bool& b)
{
    m_type = "boolean";
    m_value = b ? "true" : "false";
}

template <>
inline void MetadataNodeImpl::setValue(const std::string& s)
{
    m_type = "string";
    m_value = s;
}

template <>
inline void MetadataNodeImpl::setValue(const char * const & c)
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
inline void MetadataNodeImpl::setValue(const float& f)
{
    m_type = "float";
    m_value = Utils::toString(f);
}

template <>
inline void MetadataNodeImpl::setValue<double>(const double& d)
{
    m_type = "double";

    // Get rid of -0.
    double dd = d;
    if (dd == 0.0)
        dd = 0.0;
    m_value = Utils::toString(dd);
}

template <>
inline void MetadataNodeImpl::setValue(const SpatialReference& ref)
{
    m_type = "spatialreference";
    m_value = Utils::toString(ref);
}

template <>
inline void MetadataNodeImpl::setValue(const BOX3D& b)
{
    m_type = "bounds";
    m_value = Utils::toString(b);
}

template <>
inline void MetadataNodeImpl::setValue(const unsigned char& u)
{
    m_type = "nonNegativeInteger";
    m_value = Utils::toString(u);
}

template <>
inline void MetadataNodeImpl::setValue(const unsigned short& u)
{
    m_type = "nonNegativeInteger";
    m_value = Utils::toString(u);
}

template <>
inline void MetadataNodeImpl::setValue(const unsigned int& u)
{
    m_type = "nonNegativeInteger";
    m_value = Utils::toString(u);
}

template <>
inline void MetadataNodeImpl::setValue(const unsigned long& u)
{
    m_type = "nonNegativeInteger";
    m_value = Utils::toString(u);
}

template <>
inline void MetadataNodeImpl::setValue(const unsigned long long& u)
{
    m_type = "nonNegativeInteger";
    m_value = Utils::toString(u);
}

template <>
inline void MetadataNodeImpl::setValue(const char& i)
{
    m_type = "integer";
    m_value = Utils::toString(i);
}

template <>
inline void MetadataNodeImpl::setValue(const signed char& i)
{
    m_type = "integer";
    m_value = Utils::toString(i);
}

template <>
inline void MetadataNodeImpl::setValue(const short& s)
{
    m_type = "integer";
    m_value = Utils::toString(s);
}

template <>
inline void MetadataNodeImpl::setValue(const int& i)
{
    m_type = "integer";
    m_value = Utils::toString(i);
}

template <>
inline void MetadataNodeImpl::setValue(const long& l)
{
    m_type = "integer";
    m_value = Utils::toString(l);
}

template <>
inline void MetadataNodeImpl::setValue(const long long& l)
{
    m_type = "integer";
    m_value = Utils::toString(l);
}

template <>
inline void MetadataNodeImpl::setValue(const boost::uuids::uuid& u)
{
    m_type = "uuid";
    m_value = Utils::toString(u);
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

    MetadataNode clone(const std::string& name)
    {
        MetadataNode node;
        node.m_impl.reset(new MetadataNodeImpl(*m_impl));
        node.m_impl->m_name = name;
        return node;
    }

    MetadataNode add(MetadataNode node)
        { return MetadataNode(m_impl->add(node.m_impl)); }

    MetadataNode addList(MetadataNode node)
        { return MetadataNode(m_impl->addList(node.m_impl)); }

    MetadataNode addEncoded(const std::string& name,
        const unsigned char *buf, size_t size,
        const std::string& descrip = std::string())
    {
        MetadataNodeImplPtr impl = m_impl->add(name);
        impl->setValue(Utils::base64_encode(buf, size));
        impl->m_type = "base64Binary";
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

    MetadataNode addListEncoded(const std::string& name,
        const unsigned char *buf, size_t size,
        const std::string& descrip = std::string())
    {
        MetadataNodeImplPtr impl = m_impl->addList(name);
        impl->setValue(Utils::base64_encode(buf, size));
        impl->m_type = "base64Binary";
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

    MetadataNode addWithType(const std::string& name, const std::string& value,
        const std::string& type, const std::string& descrip)
    {
        MetadataNodeImplPtr impl = m_impl->add(name);
        impl->m_type = type;
        impl->m_value = value;
        impl->m_descrip = descrip;
        return MetadataNode(impl);
    }

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

    MetadataType::Enum kind() const
        { return m_impl->m_kind; }

    std::string name() const
        { return m_impl->m_name; }

    template<typename T>
    T value() const
    {
        T t;

        if (m_impl->m_type == "base64Binary")
        {
            std::vector<uint8_t> encVal =
                Utils::base64_decode(m_impl->m_value);
            encVal.resize(sizeof(T));
            memcpy(&t, encVal.data(), sizeof(T));
        }
        else
        {
            if (!Utils::fromString<T>(m_impl->m_value, t))
            {
                // Static to get default initialization.
                static T t2;
                std::cerr << "Error converting metadata [" << name() <<
                    "] = " << m_impl->m_value << " to type " <<
                    Utils::typeidName<T>() << " -- return default initialized.";
                t = t2;
            }
        }
        return t;
    }

    std::string value() const
        { return m_impl->m_value; }

    std::string jsonValue() const
    {
        std::string val;
        if (m_impl->m_type == "string" || m_impl->m_type == "base64Binary" ||
            m_impl->m_type == "uuid")
        {
            std::string val("\"");
            val += escapeQuotes(value()) + "\"";
            return val;
        }
        return value();
    }

    std::string description() const
        { return m_impl->m_descrip; }

    MetadataNodeList children() const
    {
        MetadataNodeList outnodes;

        const MetadataSubnodes& nodes = m_impl->m_subnodes;
        for (auto si = nodes.begin(); si != nodes.end(); ++si)
        {
            const MetadataImplList& l = si->second;
            for (auto li = l.begin(); li != l.end(); ++li)
                outnodes.push_back(MetadataNode(*li));
        }
        return outnodes;
    }

    MetadataNodeList children(const std::string& name) const
    {
        MetadataNodeList outnodes;

        auto si = m_impl->m_subnodes.find(name);
        if (si != m_impl->m_subnodes.end())
        {
            const MetadataImplList& l = si->second;
            for (auto li = l.begin(); li != l.end(); ++li)
                outnodes.push_back(MetadataNode(*li));
        }
        return outnodes;
    }

    bool hasChildren() const
        { return m_impl->m_subnodes.size(); }

    std::vector<std::string> childNames() const
    {
        std::vector<std::string> names;

        MetadataSubnodes& nodes = m_impl->m_subnodes;
        for (auto si = nodes.begin(); si != nodes.end(); ++si)
            names.push_back(si->first);
        return names;
    }

    bool operator ! ()
        { return empty(); }
    bool valid() const
        { return !empty(); }
    bool empty() const
        { return m_impl->m_name.empty() && !hasChildren(); }

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
    MetadataNodeList findChildren(PREDICATE p)
    {
        MetadataNodeList matches;

        auto nodes = children();
        for (auto ai = nodes.begin(); ai != nodes.end(); ++ai)
        {
            MetadataNode& n = *ai;
            if (p(n))
                matches.push_back(n);
        }
        return matches;
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

private:
    MetadataNodeImplPtr m_impl;

    MetadataNode(MetadataNodeImplPtr node) : m_impl(node)
        {}

    std::string escapeQuotes(const std::string& in) const
    {
        std::string out;
        for (std::size_t i(0); i < in.size(); ++i)
        {
            if (in[i] == '"' && ((i && in[i - 1] != '\\') || !i))
            {
                out.push_back('\\');
            }
            out.push_back(in[i]);
        }
        return out;
    }
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
    friend class BasePointTable;

public:
    Metadata() : m_root("root"), m_private("private")
    {}

    Metadata(const std::string& name) : m_name(name)
    {}

    MetadataNode getNode() const
        { return m_root; }

    inline static std::string inferType(const std::string& val);
private:
    MetadataNode m_root;
    MetadataNode m_private;
    std::string m_name;
};
typedef std::shared_ptr<Metadata> MetadataPtr;

inline std::string Metadata::inferType(const std::string& val)
{
    size_t pos;

    long l;
    try
    {
        pos = 0;
        l = std::stol(val, &pos);
    }
    catch (std::invalid_argument)
    {}
    if (pos == val.length())
        return (l < 0 ? "nonNegativeInteger" : "integer");

    try
    {
        pos = 0;
        std::stod(val, &pos);
    }
    catch(std::invalid_argument)
    {}

    if (pos == val.length())
        return "double";

    BOX2D b2d;
    std::istringstream iss1(val);
    iss1 >> b2d;
    if (iss1.good())
        return "bounds";

    BOX3D b3d;
    std::istringstream iss2(val);
    iss2 >> b3d;
    if (iss2.good())
        return "bounds";

    if (val == "true" || val == "false")
        return "boolean";

    try
    {
        SpatialReference s(val);
        return "spatialreference";
    }
    catch (pdal_error&)
    {
    }

    boost::uuids::uuid uuid;
    std::istringstream iss3(val);
    iss3 >> uuid;
    if (iss3.good())
        return "uuid";

    return "string";
}

} // namespace pdal


