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

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <io/FauxReader.hpp>

#include <pdal/PluginHelper.hpp>


namespace pdal
{

static StaticPluginInfo const s_info{
    "readers.XYZTimeFauxReader",
    "XYZ time Faux Reader",
    "none"
};


class PDAL_DLL XYZTimeFauxReader : public Reader, public Streamable
{
public:
    XYZTimeFauxReader() {};

    std::string getName() const
    {
        return s_info.name;
    };

private:
    using urd = std::uniform_real_distribution<double>;
    std::mt19937 m_generator;
    int m_numReturns;
    point_count_t m_index;
    std::unique_ptr<urd> m_uniformX;
    std::unique_ptr<urd> m_uniformY;
    std::unique_ptr<urd> m_uniformZ;
    uint16_t m_time;
    BOX4D m_bounds;

    virtual void addArgs(ProgramArgs& args)
    {
        args.add("bounds", "X/Y/Z/time limits", m_bounds, BOX4D(0., 0., 0., 0., 1., 1., 1., 1));
        args.add("number_of_returns", "Max number of returns", m_numReturns, 1);
    }

    virtual void prepared(PointTableRef table)
    {
        if (!m_countArg->set())
            throwError("Argument 'count' needs a value and none was provided.");
        if (m_numReturns > 10)
            throwError("Option 'number_of_returns' must be in the range [0,10].");
    }

    virtual void initialize()
    {
        m_generator.seed((uint32_t)std::time(NULL));
        m_uniformX.reset(new urd(m_bounds.minx, m_bounds.maxx));
        m_uniformY.reset(new urd(m_bounds.miny, m_bounds.maxy));
        m_uniformZ.reset(new urd(m_bounds.minz, m_bounds.maxz));
    }
    virtual void addDimensions(PointLayoutPtr layout)
    {
        Dimension::IdList ids = {
            Dimension::Id::X,
            Dimension::Id::Y,
            Dimension::Id::Z,
            Dimension::Id::GpsTime,
            Dimension::Id::Density
        };
        layout->registerDims(ids);
    }

    virtual void ready(PointTableRef table)
    {
        m_time = 0;
        m_index = 0;
    }

#pragma warning(push)
#pragma warning(disable : 4244)

    virtual bool processOne(PointRef& point)
    {
       double x(0);
       double y(0);
       double z(0);
       double tm(0);
       double density = 1.0;

       if (m_index >= m_count)
           return false;

       x = (*m_uniformX)(m_generator);
       y = (*m_uniformY)(m_generator);
       z = (*m_uniformZ)(m_generator);
       if (m_count > 1)
           tm = m_bounds.mintm + ((m_bounds.maxtm - m_bounds.mintm) / (m_count - 1)) * m_index;
       else
           tm = m_bounds.mintm;


       std::cout << "[" << " "
                 << x << ", " << y << ", " << z << ", " << tm
                 << " " << "]"
                 << std::endl;

       point.setField(Dimension::Id::X, x);
       point.setField(Dimension::Id::Y, y);
       point.setField(Dimension::Id::Z, z);
       point.setField(Dimension::Id::GpsTime, tm);
       point.setField(Dimension::Id::Density, density);
       m_index++;
       return true;
    }

#pragma warning(pop)

    virtual point_count_t read(PointViewPtr view, point_count_t count)
    {
        for (PointId idx = 0; idx < count; ++idx) {
            PointRef point = view->point(idx);
            if (!processOne(point))
                break;
            if (m_cb)
                m_cb(*view, idx);
        };
        return count;
    }

    virtual bool eof()
        { return false; }

    XYZTimeFauxReader& operator=(const XYZTimeFauxReader&);
    XYZTimeFauxReader(const XYZTimeFauxReader&);
    };

CREATE_STATIC_STAGE(XYZTimeFauxReader, s_info)

}