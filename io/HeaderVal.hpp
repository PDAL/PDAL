/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/util/Uuid.hpp>

namespace pdal
{

template <typename T>
class BaseHeaderVal
{
protected:
    T m_val;
    T m_defVal;
    bool m_valSet;

    BaseHeaderVal() : m_val(T()), m_defVal(T()), m_valSet(false)
    {}

    BaseHeaderVal(const T& t) : m_val(T()), m_defVal(t), m_valSet(false)
    {}

public:
    bool valSet() const
        { return m_valSet; }

virtual void print(const std::string& s)
{}
};

template <typename T, T MIN = std::numeric_limits<T>::lowest(),
    T MAX = (std::numeric_limits<T>::max)()>
class NumHeaderVal : public BaseHeaderVal<T>
{
public:
    typedef T type;

    NumHeaderVal()
    {}

    NumHeaderVal(const T& t) : BaseHeaderVal<T>(t)
    {}

    bool setVal(T val)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (val >= MIN && val <= MAX)
        {
            this->m_val = val;
            this->m_valSet = true;
            return true;
        }
        return false;
#pragma GCC diagnostic pop
    }

    T val() const
        { return (this->m_valSet ? this->m_val : this->m_defVal); }
};


template <typename T, T MIN, T MAX>
inline std::istream& operator>>(std::istream& in, NumHeaderVal<T, MIN, MAX>& h)
{
    std::string s;
    T t;

    in >> s;
    if (!Utils::fromString(s, t) || !h.setVal(t))
        in.setstate(std::ios::failbit);
    return in;
}


template<typename T, T MIN, T MAX>
inline std::ostream& operator<<(std::ostream& out,
    const NumHeaderVal<T, MIN, MAX>& h)
{
    out << Utils::toString(h.val());
    return out;
}


class DoubleHeaderVal : public BaseHeaderVal<double>
{
public:
    typedef double type;

    DoubleHeaderVal()
    {}

    DoubleHeaderVal(const double& d) : BaseHeaderVal(d)
    {}

    bool setVal(double val)
    {
        m_val = val;
        m_valSet = true;
        return true;
    }

    double val()
        { return (m_valSet ? m_val : m_defVal); }
};

inline std::istream& operator>>(std::istream& in, DoubleHeaderVal& h)
{
    double d;
    in >> d;
    if (!h.setVal(d))
        in.setstate(std::ios::failbit);
    return in;
}

template <size_t LEN>
class StringHeaderVal : public BaseHeaderVal<std::string>
{
public:
    typedef std::string type;

    StringHeaderVal()
    {}

    StringHeaderVal(const std::string& s) : BaseHeaderVal(s)
    {}

    bool setVal(std::string val)
    {
        m_valSet = true;
        m_val = val;
        if (LEN > 0)
            m_val.resize((std::min)(m_val.length(), LEN));
        return (LEN == 0 || val.length() <= LEN);
    }

    std::string val() const
        { return m_valSet ? m_val : m_defVal; }
};

template <size_t LEN>
inline std::istream& operator>>(std::istream& in, StringHeaderVal<LEN>& h)
{
    std::string s;
    in >> s;
    if (!h.setVal(s))
        in.setstate(std::ios::failbit);
    return in;
}

template <size_t LEN>
inline std::ostream& operator<<(std::ostream& out,
    const StringHeaderVal<LEN>& h)
{
    out << h.val();
    return out;
}

class UuidHeaderVal : public BaseHeaderVal<Uuid>
{
public:
    typedef Uuid type;

    UuidHeaderVal()
    {}

    UuidHeaderVal(const Uuid& uuid) : BaseHeaderVal(uuid)
    {}

    bool setVal(Uuid val)
    {
        m_valSet = true;
        m_val = val;
        return true;
    }

    Uuid val() const
        { return m_valSet ? m_val : m_defVal; }
};

inline std::istream& operator>>(std::istream& in, UuidHeaderVal& h)
{
    Uuid u;
    in >> u;
    if (!h.setVal(u))
        in.setstate(std::ios::failbit);
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const UuidHeaderVal& h)
{
    out << h.val();
    return out;
}

} // namespace pdal
