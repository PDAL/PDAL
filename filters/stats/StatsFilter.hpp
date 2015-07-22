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

#include <pdal/Filter.hpp>

extern "C" int32_t StatsFilter_ExitFunc();
extern "C" PF_ExitFunc StatsFilter_InitPlugin();

namespace pdal
{
namespace stats
{

class PDAL_DLL Summary
{
public:
    enum EnumType
    {
        NoEnum,
        Enumerate,
        Count
    };

typedef std::map<double, point_count_t> EnumMap;

public:
    Summary(std::string name, EnumType enumerate) :
        m_name(name), m_enumerate(enumerate)
    { reset(); }

    double minimum() const
        { return m_min; }
    double maximum() const
        { return m_max; }
    double average() const
        { return m_avg; }
    point_count_t count() const
        { return m_cnt; }
    std::string name() const
        { return m_name; }
    const EnumMap& values() const
        { return m_values; }

    void extractMetadata(MetadataNode &m) const;

    void reset()
    {
        m_max = (std::numeric_limits<double>::lowest)();
        m_min = (std::numeric_limits<double>::max)();
        m_cnt = 0;
        m_avg = 0.0;
    }

    void insert(double value)
    {
        m_cnt++;
        m_min = (std::min)(m_min, value);
        m_max = (std::max)(m_max, value);
        m_avg += (value - m_avg) / m_cnt;
        if (m_enumerate != NoEnum)
            m_values[value]++;
    }

private:
    std::string m_name;
    EnumType m_enumerate;
    double m_max;
    double m_min;
    double m_avg;
    EnumMap m_values;
    point_count_t m_cnt;
};

} // namespace stats

// This is just a pass-through filter, which collects some stats about
// the points that are fed through it
class PDAL_DLL StatsFilter : public Filter
{
public:
    StatsFilter() : Filter()
        {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    const stats::Summary& getStats(Dimension::Id::Enum d) const;
    void reset();

private:
    StatsFilter& operator=(const StatsFilter&); // not implemented
    StatsFilter(const StatsFilter&); // not implemented
    virtual void processOptions(const Options& options);
    virtual void prepared(PointTableRef table);
    virtual void done(PointTableRef table);
    virtual void filter(PointView& view);
    void extractMetadata();

    StringList m_dimNames;
    StringList m_enums;
    StringList m_counts;
    std::map<Dimension::Id::Enum, stats::Summary> m_stats;
};

} // namespace pdal
