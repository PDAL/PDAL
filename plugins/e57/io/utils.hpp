#ifndef _UTILS_H_
#define _UTILS_H_
#include <string>
#include <E57Format.h>
#include <pdal/Dimension.hpp>

namespace pdal
{
namespace e57plugin
{
    // converts an e57 dimension string to a pdal dimension
    // returns pdal::Dimension::Id::Unknown in case the dimension is not recognised
    pdal::Dimension::Id e57ToPdal(std::string e57Dimension);

    // converts a pdal dimension to the corresponding E57 string
    // returns an empty string in case the dimension is not recognised
    std::string pdalToE57(pdal::Dimension::Id pdalDimension);

    /// Converts a value from E57 to pdal. Handles change in type representation
    /// For example, intensity in e57 is between 0 and 1 and 0 and 2^16 in pdal
    double rescaleE57ToPdalValue(std::string e57Dimension, double value,const std::pair<double,double>& e57Bounds);

    std::vector<pdal::Dimension::Id> supportedPdalTypes();
    std::vector<std::string> supportedE57Types();

    // Tries to find the limit of a dimension in the e57 node headers
    // return nan if not found
    std::pair<double,double> getLimits(const e57::StructureNode &prototype, std::string fieldName);

    std::string getDescription(const e57::Node &node);
    e57::Node getNode(const e57::Node &parent, std::string childName); // TODO remove if never used

    // Get the bounds of a given dimension as expected by pdal
    std::pair<double,double> getPdalBounds(pdal::Dimension::Id id);
}
}

#endif // _UTILS_H_

