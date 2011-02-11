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

#include <limits>

#include "libpc/Utils.hpp"

namespace libpc
{

template <typename T>
class LIBPC_DLL Range
{
public:
    T minimum;
    T maximum;

    typedef T value_type;

    Range(T mmin=(std::numeric_limits<T>::max)(), T mmax=(std::numeric_limits<T>::min)())
        : minimum(mmin), maximum(mmax) {}


    Range(Range const& other)
        : minimum(other.minimum)
        , maximum(other.maximum)
    {
    }

    Range& operator=(Range<T> const& rhs)
    {
        if (&rhs != this)
        {
            minimum = rhs.minimum;
            maximum = rhs.maximum;
        }
        return *this;
    }

    bool operator==(Range<T> const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(Range const& rhs) const
    {
        return !(equal(rhs));
    }

    bool equal(Range const& other) const
    {
        if (!(Utils::compare_distance(minimum, other.minimum))
                || !(Utils::compare_distance(maximum, other.maximum)))
        {
            return false;
        }

        return true;
    }

    bool overlaps(Range const& r) const
    {
        return minimum <= r.maximum && maximum >= r.minimum;
    }

    bool contains(Range const& r) const
    {
        return minimum <= r.minimum && r.maximum <= maximum;
    }

    bool contains(T v) const
    {
        return minimum <= v && v <= maximum;
    }

    bool empty(void) const
    {
        return Utils::compare_distance(minimum, (std::numeric_limits<T>::max)()) && Utils::compare_distance(maximum, (std::numeric_limits<T>::min)());
    }

    void shift(T v)
    {
        minimum += v;
        maximum += v;
    }

    void scale(T v)
    {
        minimum *= v;
        maximum *= v;
    }

    void clip(Range const& r)
    {
        if (r.minimum > minimum)
            minimum = r.minimum;
        if (r.maximum < maximum)
            maximum = r.maximum;
    }

    void grow(T v)
    {
        if (v < minimum)
            minimum = v;
        if (v > maximum)
            maximum = v;
    }

    void grow(Range const& r)
    {
        grow(r.minimum);
        grow(r.maximum);
    }

    T length() const
    {
        return maximum - minimum;
    }
};

} // namespace libpc

