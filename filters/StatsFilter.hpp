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
#include <pdal/Streamable.hpp>

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
        Count,
        Global
    };

typedef std::map<double, point_count_t> EnumMap;
typedef std::vector<double> DataVector;

public:
    Summary(std::string name, EnumType enumerate, bool advanced = true) :
        m_name(name), m_enumerate(enumerate), m_advanced(advanced)
    { reset(); }

    // Merge another summary with this one. 'name', 'enumerate' and 'advanced' must match
    // or false is returned and no merge occurs.
    bool merge(const Summary& s);
    double minimum() const
        { return m_min; }
    double maximum() const
        { return m_max; }
    double average() const
        { return M1; }
    double populationVariance() const
        { return M2 / m_cnt; }
    double sampleVariance() const
        { return M2 / (m_cnt - 1.0); }
    double variance() const
        { return sampleVariance(); }
    double populationStddev() const
        { return std::sqrt(populationVariance()); }
    double sampleStddev() const
        { return std::sqrt(sampleVariance()); }
    double stddev() const
        { return sampleStddev(); }
    double populationSkewness() const
    {
        if (!M2 || ! m_advanced)
            return 0;
        return std::sqrt(double(m_cnt)) * M3 / std::pow(M2, 1.5);
    }
    double sampleSkewness() const
    {
        if (M2 == 0 || m_cnt <= 2 || !m_advanced)
            return 0.0;
        double c((double)m_cnt);
        return populationSkewness() * std::sqrt(c) * std::sqrt(c - 1) / (c - 2);
    }
    double skewness() const
    {
        return sampleSkewness();
    }
    double populationKurtosis() const
    {
        if (M2 == 0 || !m_advanced)
            return 0;
        return double(m_cnt) * M4 / (M2 * M2);
    }
    double populationExcessKurtosis() const
    {
        if (M2 == 0 || !m_advanced)
            return 0;
        return populationKurtosis() - 3;
    }
    double sampleKurtosis() const
    {
        if (M2 == 0 || m_cnt <= 3 || !m_advanced)
            return 0;
        double c((double)m_cnt);
        return populationKurtosis() * (c + 1) * (c - 1) / ((c - 2) * (c - 3));
    }
    double sampleExcessKurtosis() const
    {
        if (M2 == 0 || m_cnt <= 3 || !m_advanced)
            return 0;
        double c((double)m_cnt);
        return sampleKurtosis() - 3 * (c - 1) * (c - 1) / ((c - 2) * (c - 3));
    }
    double kurtosis() const
    {
        return sampleExcessKurtosis();
    }
    double median() const
        { return m_median; }
    double mad() const
        { return m_mad; }
    point_count_t count() const
        { return m_cnt; }
    std::string name() const
        { return m_name; }
    const EnumMap& values() const
        { return m_values; }

    void extractMetadata(MetadataNode &m);
    void computeGlobalStats();

    void reset()
    {
        m_max = (std::numeric_limits<double>::lowest)();
        m_min = (std::numeric_limits<double>::max)();
        m_cnt = 0;
        m_median = 0.0;
        m_mad = 0.0;
        M1 = M2 = M3 = M4 = 0.0;
    }

    void insert(double value)
    {
        m_cnt++;
        m_min = (std::min)(m_min, value);
        m_max = (std::max)(m_max, value);

        if (m_enumerate != NoEnum)
            m_values[value]++;
        if (m_enumerate == Global)
        {
            if (m_data.capacity() - m_data.size() < 10000)
                m_data.reserve(m_data.capacity() + m_cnt);
            m_data.push_back(value);
        }

        // stolen from http://www.johndcook.com/blog/skewness_kurtosis/

        point_count_t n(m_cnt);

        // Difference from the mean
        double delta = value - M1;
        // Portion that this point's difference from the mean contributes
        // to the mean.
        double delta_n = delta / n;
        double term1 = delta * delta_n * (n - 1);

        // First moment - average.
        M1 += delta_n;

        if (m_advanced)
        {
            double delta_n2 = pow(delta_n, 2.0);
            // Fourth moment - kurtosis (sum part)
            M4 += term1 * delta_n2 * (n*n - 3*n + 3) +
                (6 * delta_n2 * M2) - (4 * delta_n * M3);
            // Third moment - skewness (sum part)
            M3 += term1 * delta_n * (n - 2) - 3 * delta_n * M2;
        }
        // Second moment - variance (sum part)
        M2 += term1;
    }

private:
    std::string m_name;
    EnumType m_enumerate;
    bool m_advanced;
    double m_max;
    double m_min;
    double m_mad;
    double m_median;
    EnumMap m_values;
    DataVector m_data;
    point_count_t m_cnt;
    double M1, M2, M3, M4;
};

} // namespace stats

// This is just a pass-through filter, which collects some stats about
// the points that are fed through it
class PDAL_DLL StatsFilter : public Filter, public Streamable
{
public:
    StatsFilter()
        {}

    std::string getName() const;

    const stats::Summary& getStats(Dimension::Id d) const;
    void reset();

private:
    StatsFilter& operator=(const StatsFilter&); // not implemented
    StatsFilter(const StatsFilter&); // not implemented
    virtual void addArgs(ProgramArgs& args);
    virtual bool processOne(PointRef& point);
    virtual void prepared(PointTableRef table);
    virtual void done(PointTableRef table);
    virtual void filter(PointView& view);
    void extractMetadata(PointTableRef table);

    StringList m_dimNames;
    StringList m_enums;
    StringList m_counts;
    StringList m_global;
    bool m_advanced;
    std::map<Dimension::Id, stats::Summary> m_stats;
};

} // namespace pdal
