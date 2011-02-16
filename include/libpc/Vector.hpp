/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_VECTOR_HPP
#define INCLUDED_VECTOR_HPP

#include <limits>

#include "libpc/Utils.hpp"

namespace libpc
{

template <typename T>
class LIBPC_DLL Vector
{
private:
    std::vector<T> m_data;

public:
    typedef T value_type;

    Vector()
    {
        return;
    }

    Vector(T v0)
    {
        m_data.push_back(v0);
    }

    Vector(T v0, T v1)
    {
        m_data.push_back(v0);
        m_data.push_back(v1);
    }

    Vector(T v0, T v1, T v2)
    {
        m_data.push_back(v0);
        m_data.push_back(v1);
        m_data.push_back(v2);
    }

    Vector(std::vector<T> v)
    {
        m_data = v;
    }

    Vector& operator=(Vector<T> const& rhs)
    {
        if (&rhs != this)
        {
            m_data = rhs.m_data;
        }
        return *this;
    }

    bool operator==(Vector<T> const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(Vector const& rhs) const
    {
        return !(equal(rhs));
    }

    T get0() const
    {
        assert(m_data.size() >= 0);
        return m_data[0];
    }

    T get1() const
    {
        assert(m_data.size() >= 1);
        return m_data[1];
    }

    T get2() const
    {
        assert(m_data.size() >= 2);
        return m_data[2];
    }

    T getn(std::size_t n) const
    {
        assert(m_data.size() >= n);
        return m_data[n];
    }

    void set(T t)
    {
        m_data.resize(1);
        m_data[0] = t;
    }

    void set(T t0, T t1)
    {
        m_data.resize(2);
        m_data[0] = t0;
        m_data[1] = t1;
    }

    void set(T t0, T t1, T t2)
    {
        m_data.resize(3);
        m_data[0] = t0;
        m_data[1] = t1;
        m_data[2] = t2;
    }

    void set(std::vector<T> v)
    {
        m_data.resize(v.size());
        m_data = v;
    }

    bool equal(Vector const& other) const
    {
        for (std::size_t i=0; i<m_data.size(); i++)
        {
            if (!Utils::compare_distance(m_data[i], other.m_data[i]))
            {
                return false;
            }
        }

        return true;
    }

    void shift(T v)
    {
        for (std::size_t i=0; i<m_data.size(); i++)
        {
            m_data[i] += v;
        }
    }

    void scale(T v)
    {
        for (std::size_t i=0; i<m_data.size(); i++)
        {
            m_data[i] *= v;
        }
    }

    std::size_t size() const
    {
        return m_data.size();
    }
};


template<class T>
std::ostream& operator<<(std::ostream& ostr, const Vector<T>& vector)
{
    ostr << "(";
    for (size_t d = 0; d < vector.size(); ++d)
    {
        ostr << vector.vN(i);
        if (i<vector.size()-1)
            ostr << ",";
    }
    ostr << ")";
    return ostr;
}


} // namespace libpc

#endif
