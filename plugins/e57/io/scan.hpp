#ifndef _SCAN_HPP_
#define _SCAN_HPP_

#include <pdal/pdal_types.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Dimension.hpp>
#include <E57Format.h>

namespace e57
{
class Scan
{

public:
    Scan(const e57::StructureNode& scanNode);

    pdal::point_count_t getNumPoints() const;

    /// Get the pdal dimensions that can be read from this scan
    std::set<std::string> getDimensions() const;

    e57::CompressedVectorNode getPoints() const;

    std::pair<double,double> getLimits(pdal::Dimension::Id pdalId) const;

    bool hasPose() const;
    void transformPoint(pdal::PointRef pt) const;

private:
    /// Called only once on constructor called
    void decodeHeader_();

    void getPose_();

    // Core data holders for underlying e57 object
    std::unique_ptr<e57::StructureNode> m_rawData;
    std::unique_ptr<e57::CompressedVectorNode> m_rawPoints;
    pdal::point_count_t m_numPoints;

    // supported configs
    std::set<std::string> m_e57TypeToPdalDimension;

    // field limits in header
    std::map<pdal::Dimension::Id,std::pair<double,double>> m_valueBounds;

    // Pose information
    double m_translation[3] = {0};
    double m_rotation[3][3] = {0}; 
    bool m_hasPose = false;
};
}
#endif // _SCAN_HPP_
