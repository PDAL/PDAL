/******************************************************************************
 * $Id$
 *
 * Project:  PDAL - http://pdal.org - A BSD library for point cloud data.
 * Purpose:  PDAL chipper class
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2011, Howard Butler
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

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/PointView.hpp>

#include <vector>

extern "C" int32_t ChipperFilter_ExitFunc();
extern "C" PF_ExitFunc ChipperFilter_InitPlugin();

namespace pdal
{

class Stage;


class PDAL_DLL ChipperFilter;

enum Direction
{
    DIR_X,
    DIR_Y,
    DIR_NONE
};


class PDAL_DLL ChipPtRef
{
    friend class ChipRefList;
    friend class ChipperFilter;

private:
    double m_pos;
    point_count_t m_ptindex;
    uint32_t m_oindex;

public:
    bool operator < (const ChipPtRef& pt) const
    {
        return m_pos < pt.m_pos;
    }
};


class PDAL_DLL ChipRefList
{
    friend class ChipperFilter;

private:
    std::vector<ChipPtRef> m_vec;
    Direction m_dir;

    ChipRefList(Direction dir = DIR_NONE) : m_dir(dir)
    {}
    std::vector<ChipPtRef>::size_type size() const
    {
        return m_vec.size();
    }
    void reserve(std::vector<ChipPtRef>::size_type n)
    {
        m_vec.reserve(n);
    }
    void resize(std::vector<ChipPtRef>::size_type n)
    {
        m_vec.resize(n);
    }
    void push_back(const ChipPtRef& ref)
    {
        m_vec.push_back(ref);
    }
    std::vector<ChipPtRef>::iterator begin()
    {
        return m_vec.begin();
    }
    std::vector<ChipPtRef>::iterator end()
    {
        return m_vec.end();
    }
    ChipPtRef& operator[](uint32_t pos)
    {
        return m_vec[pos];
    }
    std::string Dir()
    {
        if (m_dir == DIR_X)
            return "X";
        else if (m_dir == DIR_Y)
            return "Y";
        else
            return "NONE";
    }
};


class PDAL_DLL ChipperFilter : public pdal::Filter
{
public:
    ChipperFilter() : Filter(),
        m_xvec(DIR_X), m_yvec(DIR_Y), m_spare(DIR_NONE)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    virtual void processOptions(const Options& options);
    virtual PointViewSet run(PointViewPtr view);

    void load(PointView& view, ChipRefList& xvec,
        ChipRefList& yvec, ChipRefList& spare);
    void partition(point_count_t size);
    void decideSplit(ChipRefList& v1, ChipRefList& v2,
        ChipRefList& spare, PointId left, PointId right);
    void split(ChipRefList& wide, ChipRefList& narrow,
        ChipRefList& spare, PointId left, PointId right);
    void finalSplit(ChipRefList& wide, ChipRefList& narrow,
        PointId pleft, PointId pcenter);
    void emit(ChipRefList& wide, PointId widemin, PointId widemax);

    PointId m_threshold;
    PointViewPtr m_inView;
    PointViewSet m_outViews;
    std::vector<PointId> m_partitions;
    ChipRefList m_xvec;
    ChipRefList m_yvec;
    ChipRefList m_spare;

    ChipperFilter& operator=(const ChipperFilter&); // not implemented
    ChipperFilter(const ChipperFilter&); // not implemented
};

} // namespace pdal

