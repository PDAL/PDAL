/******************************************************************************
 *
 * Project:  libPC - http://libpc.org - A BSD library for point clouds
 * Purpose:  swig/python bindings for libpc
 * Author:   Pete J. Gadomski (pete.gadomski@gmail.com)
 *
 ******************************************************************************
 * Copyright (c) 2011, Pete J. Gadomski (pete.gadomski@gmail.com)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
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
 
%module libpc

%{
#include <iostream>

#include "libpc/libpc_config.hpp"

#include "libpc/Bounds.hpp"

%}

namespace libpc
{

template <typename T>
class Range
{
public:
    typedef T value_type;

    Range();
    Range(T minimum, T maximum);
    T getMinimum() const;
    void setMinimum(T value);
    T getMaximum() const;
    void setMaximum(T value);
    bool equal(Range const& other) const;
    bool overlaps(Range const& r) const;
    bool contains(Range const& r) const;
    bool contains(T v) const;
    bool empty(void) const;
    void clip(Range const& r);
    void grow(T v);
    void grow(Range const& r);
    void grow(T lo, T hi);
    T length() const;
};

%rename(Range_double) Range<double>;
%template(Range_double) Range<double>;

template <typename T>
class Vector
{
public:
    Vector();
    Vector(T v0);
    Vector(T v0, T v1);
    Vector(T v0, T v1, T v2);
    Vector(std::vector<T> v);
    Vector(Vector<T> const& rhs);
    bool operator==(Vector<T> const& rhs) const;
    bool operator!=(Vector const& rhs) const;
    T get(std::size_t index);
    void set(std::size_t index, T v);
    void set(std::vector<T> v);
    bool equal(Vector const& other) const;
    void shift(T v);
    void scale(T v);
    std::size_t size() const;
};

%rename(Vector_double) Vector<double>;
%template(Vector_double) Vector<double>;

template <typename T>
class Bounds
{
public:
    typedef typename std::vector< Range<T> > RangeVector;
    Bounds();
    Bounds(Bounds const& other);
    Bounds(RangeVector const& ranges);
    Bounds(T minx, T miny, T minz, T maxx, T maxy, T maxz);
    Bounds(T minx, T miny, T maxx, T maxy);
    Bounds(const Vector<T>& minimum, const Vector<T>& maximum);
    T getMinimum(std::size_t const& index) const;
    void setMinimum(std::size_t const& index, T v);
    T getMaximum(std::size_t const& index) const;
    void setMaximum(std::size_t const& index, T v);
    Vector<T> getMinimum();
    Vector<T> getMaximum();
    inline bool operator==(Bounds<T> const& rhs) const;
    inline bool operator!=(Bounds<T> const& rhs) const;
    RangeVector const& dimensions() const;
    std::size_t size() const;
    bool equal(Bounds<T> const& other) const;
    bool overlaps(Bounds const& other) const;
    bool contains(Vector<T> point) const;
    bool contains(Bounds<T> const& other) const;
    void shift(std::vector<T> deltas);
    void scale(std::vector<T> deltas);
    void clip(Bounds const& r);
    void grow(Bounds const& r);
    void grow(Vector<T> const& point);
    T volume() const;
    bool empty() const;
    bool verify();
    
};

%rename(Bounds_double) Bounds<double>;
%template(Bounds_double) Bounds<double>;

}; // namespace
