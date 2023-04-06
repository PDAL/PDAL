/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc
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
 *     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <ctime>

#include <io/FauxReader.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

#include <pdal/PluginHelper.hpp>

#include "../io/Bounds4D.hpp"

namespace pdal
{

class PDAL_DLL XYZTimeFauxReader : public Reader, public Streamable
{
public:
    XYZTimeFauxReader(){};

    std::string getName() const;

private:
    using urd = std::uniform_real_distribution<double>;
    std::mt19937 m_generator;
    int m_numReturns;
    point_count_t m_index;
    std::unique_ptr<urd> m_uniformX;
    std::unique_ptr<urd> m_uniformY;
    std::unique_ptr<urd> m_uniformZ;
    std::unique_ptr<urd> m_uniformTm;
    double m_delX;
    double m_delY;
    double m_delZ;
    double m_delTm;
    double m_density;
    BOX4D m_bounds;
    Mode m_xyz_mode;
    Mode m_tm_mode;
    bool m_use_time;
    std::string m_dim4_name;

    virtual void addArgs(ProgramArgs& args);

    virtual void prepared(PointTableRef table);

    virtual void initialize();

    virtual void addDimensions(PointLayoutPtr layout);

    virtual void ready(PointTableRef table);

    virtual bool processOne(PointRef& point);

    virtual point_count_t read(PointViewPtr view, point_count_t count);

    virtual bool eof()
    {
        return false;
    }

    XYZTimeFauxReader& operator=(const XYZTimeFauxReader&);
    XYZTimeFauxReader(const XYZTimeFauxReader&);
};

} // namespace pdal