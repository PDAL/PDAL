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

namespace pdal
{

template <typename T>
class BaseHeaderVal
{
protected:
//    std::string m_name;
    T m_val;
    T m_defVal;
    bool m_forward;
    bool m_valSet;

    BaseHeaderVal() : m_forward(false), m_valSet(false)
    {}

public:
    bool valSet() const
        { return m_valSet; }

    void setDefault(const T& t)
        { m_defVal = t; }
};

template <typename T, T MIN = std::numeric_limits<T>::lowest(),
    T MAX = std::numeric_limits<T>::max()>
class NumHeaderVal : public BaseHeaderVal<T>
{
public:
    typedef T type;

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

    T val()
        { return (this->m_valSet ? this->m_val : this->m_defVal); }
};

class DoubleHeaderVal : public BaseHeaderVal<double>
{
public:
    typedef double type;

    bool setVal(double val)
    {
        m_val = val;
        m_valSet = true;
        return true;
    }

    double val()
        { return (m_valSet ? m_val : m_defVal); }
};

template <size_t LEN>
class StringHeaderVal : public BaseHeaderVal<std::string>
{
public:
    typedef std::string type;

    bool setVal(std::string val)
    {
        m_valSet = true;
        m_val = val;
        m_val.resize(std::min(m_val.length(), LEN));
        return val.length() <= LEN;
    }

    std::string val()
        { return m_valSet ? m_val : m_defVal; }
};

class UuidHeaderVal : public BaseHeaderVal<boost::uuids::uuid>
{
public:
    typedef boost::uuids::uuid type;

    bool setVal(boost::uuids::uuid val)
    {
        m_valSet = true;
        m_val = val;
        return true;
    }

    boost::uuids::uuid val()
        { return m_valSet ? m_val : m_defVal; }
};

} // namespace pdal

