
#pragma once

#include <pdal/Reader.hpp>
#include <pdal/Dimension.hpp>

#include <spz/src/cc/load-spz.h>

namespace pdal
{

class PDAL_EXPORT SpzReader : public Reader
{
public:
    SpzReader();

    std::string getName() const;
private:
    point_count_t m_numPoints;
    point_count_t m_index;
    bool m_isRemote;
    // number of spherical harmonics for each color (RGB)
    int m_numSh;
    //!! put these in a struct or something?
    Dimension::IdList m_shDims;
    Dimension::IdList m_rotDims;
    Dimension::IdList m_scaleDims;
    Dimension::IdList m_colorDims;
    Dimension::Id m_alphaDim;
    std::unique_ptr<spz::PackedGaussians> m_data;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual void done(PointTableRef table);

    void extractHeaderData();
    double extractPositions(size_t pos);
    float unpackSh(size_t pos);
    float unpackScale(size_t pos);
};

} // namespace pdal