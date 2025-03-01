
#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Writer.hpp>
#include <spz/src/cc/load-spz.h>

namespace pdal
{

class PDAL_EXPORT SpzWriter : public Writer
{
public:
    SpzWriter();
    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void prepared(PointTableRef table);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    void checkDimensions(PointLayoutPtr layout);

    bool m_antialiased;
    int m_shDegree;
    std::string m_remoteFilename;
    DimTypeList m_dims;
    std::unique_ptr<spz::PackedGaussians> m_cloud;
    //!! again, maybe keep these grouped together
    Dimension::IdList m_shDims;
    Dimension::IdList m_rotDims;
    Dimension::IdList m_scaleDims;
    Dimension::IdList m_plyColorDims;
    Dimension::Id m_plyAlphaDim;
    std::vector<PointViewPtr> m_views;
    std::string m_curFilename;
};

} // namespace pdal
