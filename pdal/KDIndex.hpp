/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/PointView.hpp>

namespace pdal
{

class KD2Impl;
class KD3Impl;
class KDFlexImpl;

class PDAL_DLL KD2Index
{
public:
    KD2Index(const PointView& buf);
    ~KD2Index();

    void build();
    PointId neighbor(double x, double y) const;
    PointId neighbor(PointId idx) const;
    PointId neighbor(PointRef &point) const;
    PointIdList neighbors(double x, double y, point_count_t k) const;
    PointIdList neighbors(PointId idx, point_count_t k) const;
    PointIdList neighbors(PointRef &point, point_count_t k) const;
    void knnSearch(double x, double y, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const;
    void knnSearch(PointId idx, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const;
    void knnSearch(PointRef& point, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const;
    PointIdList radius(double const& x, double const& y,
        double const& r) const;
    PointIdList radius(PointId idx, double const& r) const;
    PointIdList radius(PointRef &point, double const& r) const;

private:
    const PointView& m_buf;
    std::unique_ptr<KD2Impl> m_impl;
};

class PDAL_DLL KD3Index
{
public:
    KD3Index(const PointView& buf);
    ~KD3Index();

    void build();
    PointId neighbor(double x, double y, double z) const;
    PointId neighbor(PointId idx) const;
    PointId neighbor(PointRef &point) const;
    PointIdList neighbors(double x, double y, double z,
        point_count_t k, size_t stride = 1) const;
    PointIdList neighbors(PointId idx, point_count_t k,
        size_t stride = 1) const;
    PointIdList neighbors(PointRef &point, point_count_t k,
        size_t stride=1) const;
    void knnSearch(double x, double y, double z, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const;
    void knnSearch(PointId idx, point_count_t k, PointIdList *indices,
        std::vector<double> *sqr_dists) const;
    void knnSearch(PointRef &point, point_count_t k,
        PointIdList *indices, std::vector<double> *sqr_dists) const;
    PointIdList radius(double x, double y, double z, double r) const;
    PointIdList radius(PointId idx, double r) const;
    PointIdList radius(PointRef &point, double r) const;

private:
    const PointView& m_buf;
    std::unique_ptr<KD3Impl> m_impl;
};

class KDFlexIndex
{
public:
    KDFlexIndex(const PointView& buf, const Dimension::IdList& dims);
    ~KDFlexIndex();

    void build();
    PointId neighbor(PointRef &point) const;
    PointIdList neighbors(PointRef &point, point_count_t k, size_t stride = 1) const;
    PointIdList radius(PointId idx, double r) const;

private:
    const PointView& m_buf;
    const Dimension::IdList& m_dims;
    std::unique_ptr<KDFlexImpl> m_impl;
};

} // namespace pdal
