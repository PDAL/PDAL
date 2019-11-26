/******************************************************************************
* Copyright (c) 2019, Helix Re Inc.
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
*     * Neither the name of Helix Re Inc. nor the
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

#include <limits>

#include "Utils.hpp"

namespace pdal
{
namespace e57plugin
{

Dimension::Id e57ToPdal(const std::string &e57Dimension)
{
    if (e57Dimension == "cartesianX")
        return Dimension::Id::X;
    else if (e57Dimension == "cartesianY")
        return Dimension::Id::Y;
    else if (e57Dimension == "cartesianZ")
        return Dimension::Id::Z;
    else if (e57Dimension == "sphericalRange")
        return Dimension::Id::X;
    else if (e57Dimension == "sphericalAzimuth")
        return Dimension::Id::Y;
    else if (e57Dimension == "sphericalElevation")
        return Dimension::Id::Z;
    else if (e57Dimension == "nor:normalX")
        return Dimension::Id::NormalX;
    else if (e57Dimension == "nor:normalY")
        return Dimension::Id::NormalY;
    else if (e57Dimension == "nor:normalZ")
        return Dimension::Id::NormalZ;
    else if (e57Dimension == "intensity")
        return Dimension::Id::Intensity;
    else if (e57Dimension == "colorRed")
        return Dimension::Id::Red;
    else if (e57Dimension == "colorBlue")
        return Dimension::Id::Blue;
    else if (e57Dimension == "colorGreen")
        return Dimension::Id::Green;
    else if (e57Dimension == "cartesianInvalidState")
        return Dimension::Id::Omit;
    else if (e57Dimension == "sphericalInvalidState")
        return Dimension::Id::Omit;
    else if (e57Dimension == "classification")
        return Dimension::Id::Classification;
    return Dimension::Id::Unknown;
}

std::string pdalToE57(Dimension::Id pdalDimension)
{
    switch (pdalDimension)
    {
        case pdal::Dimension::Id::X:
            return "cartesianX";
        case pdal::Dimension::Id::Y:
            return "cartesianY";
        case pdal::Dimension::Id::Z:
            return "cartesianZ";
        case pdal::Dimension::Id::NormalX:
            return "nor:normalX";
        case pdal::Dimension::Id::NormalY:
            return "nor:normalY";
        case pdal::Dimension::Id::NormalZ:
            return "nor:normalZ";
        case pdal::Dimension::Id::Red:
            return "colorRed";
        case pdal::Dimension::Id::Green:
            return "colorGreen";
        case pdal::Dimension::Id::Blue:
            return "colorBlue";
        case pdal::Dimension::Id::Intensity:
            return "intensity";
        case pdal::Dimension::Id::Classification:
            return "classification";
        case pdal::Dimension::Id::Omit:
            return "cartesianInvalidState";
        default:
            return std::string();
    }
}

std::vector<Dimension::Id> supportedPdalTypes()
{
    return {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
            Dimension::Id::NormalX, Dimension::Id::NormalY, Dimension::Id::NormalZ,
            Dimension::Id::Red, Dimension::Id::Green, Dimension::Id::Blue,
            Dimension::Id::Intensity, Dimension::Id::Omit, Dimension::Id::Classification
           };
}

std::vector<std::string> supportedE57Types()
{
    return {"cartesianX",  "cartesianY", "cartesianZ",
            "nor:normalX", "nor:normalY", "nor:normalZ",
            "colorRed", "colorGreen", "colorBlue", "intensity",
            "cartesianInvalidState", "classification"};
}

std::vector<std::string> scalableE57Types()
{
    return {"colorRed", "colorGreen", "colorBlue", "intensity",
            "classification"};
}

bool getLimits(const e57::StructureNode& prototype,
               const std::string& fieldName, std::pair<double, double>& minmax)
{
    // Reset minmax
    minmax.first = minmax.second = 0;

    std::string minKey = fieldName + "Minimum";
    std::string maxKey = fieldName + "Maximum";
    std::string boundingBoxName = fieldName + "Limits";

    if (fieldName.substr(0, 5) == "color")
        boundingBoxName = "colorLimits";
    else if (fieldName[0] == 'x' || fieldName[0] == 'y' || fieldName[0] == 'z')
        boundingBoxName = "cartesianBounds";

    if (prototype.isDefined(boundingBoxName))
    {
        e57::StructureNode intbox(prototype.get(boundingBoxName));
        if (intbox.get(maxKey).type() == e57::E57_SCALED_INTEGER)
        {
            minmax.second =
                static_cast<e57::ScaledIntegerNode>(intbox.get(maxKey))
                .scaledValue();
            minmax.first =
                static_cast<e57::ScaledIntegerNode>(intbox.get(minKey))
                .scaledValue();
        }
        else if (intbox.get(maxKey).type() == e57::E57_FLOAT)
        {
            minmax.second =
                static_cast<e57::FloatNode>(intbox.get(maxKey)).value();
            minmax.first =
                static_cast<e57::FloatNode>(intbox.get(minKey)).value();
        }
        else if (intbox.get(maxKey).type() == e57::E57_INTEGER)
        {
            minmax.second = static_cast<double>(
                                static_cast<e57::IntegerNode>(intbox.get(maxKey)).value());
            minmax.first = static_cast<double>(
                               static_cast<e57::IntegerNode>(intbox.get(minKey)).value());
        }
    }
    else if (prototype.isDefined(fieldName))
    {
        if (prototype.get(fieldName).type() == e57::E57_INTEGER)
        {
            minmax.first = static_cast<double>(
                               static_cast<e57::IntegerNode>(prototype.get(fieldName))
                               .minimum());
            minmax.second = static_cast<double>(
                                static_cast<e57::IntegerNode>(prototype.get(fieldName))
                                .maximum());
        }
        else if (prototype.get(fieldName).type() == e57::E57_SCALED_INTEGER)
        {
            double scale =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .scale();
            double offset =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .offset();
            int64_t minimum =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .minimum();
            int64_t maximum =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .maximum();
            minmax.first = minimum * scale + offset;
            minmax.second = maximum * scale + offset;
        }
        else if (prototype.get(fieldName).type() == e57::E57_FLOAT)
        {
            minmax.first =
                static_cast<e57::FloatNode>(prototype.get(fieldName)).minimum();
            minmax.second =
                static_cast<e57::FloatNode>(prototype.get(fieldName)).maximum();
        }
    }
    else
        return false;
    return true;
}

// This will give positive bounds to default data type for id.
std::pair<uint64_t, uint64_t> getPdalBounds(pdal::Dimension::Id id)
{
    // pdal::Dimension::size() returns number of bytes for the pdal::Dimesion::Type.
    // eg: 1 for uint8, 2 for uint16, 4 for uint32, 8 for double, etc.
    // Max range for data type = (2 ^ (8 * no. of bytes)) - 1
    auto type = pdal::Dimension::defaultType(id);
    auto typeName = pdal::Dimension::interpretationName(type);
    if (typeName.find("uint") == 0)
    {
        auto maxVal = std::pow(2, 8 * pdal::Dimension::size(type)) - 1;
        return {0, maxVal};
    }
    throw pdal_error("Cannot retrieve bounds for : " + typeName);
}

point_count_t numPoints(const e57::VectorNode data3D)
{
    point_count_t count(0);
    int64_t scanCount = data3D.childCount();
    try
    {
        for (int scanIndex = 0; scanIndex < scanCount; scanIndex++)
        {
            e57::StructureNode scan(data3D.get(scanIndex));
            e57::CompressedVectorNode points(scan.get("points"));
            count += points.childCount();
        }
    }
    catch (e57::E57Exception& e)
    {
        e.report(__FILE__, __LINE__, __FUNCTION__);
    }
    catch (...)
    {
        throw pdal_error("Got an unknown exception");
    }
    return count;
}

void Dim::grow(double val)
{
    m_min = std::fmin(m_min, val);
    m_max = std::fmax(m_max, val);
}

void ExtraDims::addDim(std::string name, Dimension::Type type)
{
    Dim d;
    d.m_name = name;
    d.m_type = type;
    m_dimMap.push_back(d);
};

uint16_t ExtraDims::numDims()
{
    return m_dimMap.size();
}

std::vector<Dim>::iterator ExtraDims::begin()
{
    return m_dimMap.begin();
}

std::vector<Dim>::iterator ExtraDims::end()
{
    return m_dimMap.end();
}

std::vector<Dim>::iterator ExtraDims::deleteDim(std::vector<Dim>::iterator itr)
{
    return m_dimMap.erase(itr);
}

std::vector<Dim>::iterator ExtraDims::findDim(std::string name)
{
    return std::find_if(begin(), end(),
                        [name](Dim d)
    {
        return d.m_name == name;
    });
}

void ExtraDims::parse(pdal::StringList dimList)
{
    for (auto& dim : dimList)
    {
        StringList s = Utils::split2(dim, '=');
        if (s.size() != 2)
            throw pdal_error("Invalid extra dimension specified: '" + dim +
                             "'.  Need <dimension>=<type>..");
        Utils::trim(s[0]);
        Utils::trim(s[1]);
        Dimension::Type type = Dimension::type(s[1]);
        if (type == Dimension::Type::None)
        {
            throw pdal_error("Invalid extra dimension type specified: '" + dim +
                             "'.  Need <dimension>=<type>. ");

        }
        addDim(s[0], type);
    }
}
} // namespace e57plugin
} // namespace pdal

