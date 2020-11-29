/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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

#include <algorithm>

#include <pdal/FlexWriter.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

class GDALGrid;

class PDAL_DLL GDALWriter : public FlexWriter, public Streamable
{
    struct Cell
    {
        long x;
        long y;
    };
    struct Position
    {
        double x;
        double y;
    };

public:
    std::string getName() const;

    GDALWriter() : m_outputTypes(0), m_expandByPoint(true)
    {}

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void prepared(PointTableRef table);
    virtual void readyFile(const std::string& filename,
        const SpatialReference& srs);
    virtual void writeView(const PointViewPtr view);
    virtual bool processOne(PointRef& point);
    virtual void doneFile();
    void createGrid(BOX2D bounds);
    void expandGrid(BOX2D bounds);
    int width() const;
    int height() const;

    std::string m_outputFilename;
    std::string m_drivername;
    SpatialReference m_srs;
    Bounds m_bounds;
    double m_edgeLength;
    Arg *m_radiusArg;
    double m_xOrigin;
    double m_yOrigin;
    size_t m_width;
    size_t m_height;
    Arg *m_xOriginArg;
    Arg *m_yOriginArg;
    Arg *m_heightArg;
    Arg *m_widthArg;
    double m_radius;
    double m_power;
    StringList m_options;
    StringList m_outputTypeString;
    size_t m_windowSize;
    int m_outputTypes;
    std::unique_ptr<GDALGrid> m_grid;
    double m_noData;
    Dimension::Id m_interpDim;
    std::string m_interpDimString;
    Dimension::Type m_dataType;
    bool m_expandByPoint;
    bool m_fixedGrid;
    SpatialReference m_defaultSrs;
    SpatialReference m_overrideSrs;
};

}
