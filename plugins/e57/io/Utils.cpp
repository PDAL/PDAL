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

pdal::Dimension::Id pdal::e57plugin::e57ToPdal(const std::string &e57Dimension) {
    if (e57Dimension == "cartesianX")
    {
        return pdal::Dimension::Id::X;
    }
    else if (e57Dimension == "cartesianY")
    {
        return pdal::Dimension::Id::Y;
    }
    else if (e57Dimension == "cartesianZ")
    {
        return pdal::Dimension::Id::Z;
    }
    else if (e57Dimension == "sphericalRange")
    {
        return pdal::Dimension::Id::X;
    }
    else if (e57Dimension == "sphericalAzimuth")
    {
        return pdal::Dimension::Id::Y;
    }
    else if (e57Dimension == "sphericalElevation")
    {
        return pdal::Dimension::Id::Z;
    }
    else if (e57Dimension == "nor:normalX")
    {
        return pdal::Dimension::Id::NormalX;
    }
    else if (e57Dimension == "nor:normalY")
    {
        return pdal::Dimension::Id::NormalY;
    }
    else if (e57Dimension == "nor:normalZ")
    {
        return pdal::Dimension::Id::NormalZ;
    }
    else if (e57Dimension == "intensity")
    {
        return pdal::Dimension::Id::Intensity;
    }
    else if (e57Dimension == "colorRed")
    {
        return pdal::Dimension::Id::Red;
    }
    else if (e57Dimension == "colorBlue")
    {
        return pdal::Dimension::Id::Blue;
    }
    else if (e57Dimension == "colorGreen")
    {
        return pdal::Dimension::Id::Green;
    }
    else if (e57Dimension == "cartesianInvalidState")
    {
        return pdal::Dimension::Id::Omit;
    }
    else if (e57Dimension == "sphericalInvalidState")
    {
        return pdal::Dimension::Id::Omit;
    }
    else
    {
        return pdal::Dimension::Id::Unknown;
    }
}

std::string pdal::e57plugin::pdalToE57(pdal::Dimension::Id pdalDimension)
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
        case pdal::Dimension::Id::Omit:
            return "cartesianInvalidState";
        default:
            return std::string();
    }
}

std::vector<pdal::Dimension::Id> pdal::e57plugin::supportedPdalTypes()
{
    return {pdal::Dimension::Id::X,pdal::Dimension::Id::Y,pdal::Dimension::Id::Z,
        pdal::Dimension::Id::NormalX,pdal::Dimension::Id::NormalY,pdal::Dimension::Id::NormalZ,
        pdal::Dimension::Id::Red,pdal::Dimension::Id::Green,pdal::Dimension::Id::Blue,pdal::Dimension::Id::Intensity,
        pdal::Dimension::Id::Omit};
}

std::vector<std::string> pdal::e57plugin::supportedE57Types()
{
    return {"cartesianX","cartesianY","cartesianZ",
        "nor:normalX","nor:normalY","nor:normalZ",
        "colorRed","colorGreen","colorBlue","intensity",
        "cartesianInvalidState"};
}

double pdal::e57plugin::rescaleE57ToPdalValue(
        const std::string &e57Dimension, double value, const std::pair<double, double> &e57Bounds)
{
    assert(e57Bounds.second >= e57Bounds.first);
    if (e57Bounds.second == e57Bounds.first)
        return value;

    auto pdalDimension = pdal::e57plugin::e57ToPdal(e57Dimension);
    std::pair<double,double> minmax;
    try
    {
        minmax = getPdalBounds(pdalDimension);
    }
    catch (std::invalid_argument &e)
    {
        return value;
    }
    value = (value - e57Bounds.first) / (e57Bounds.second - e57Bounds.first + 1e-10);
    value = value * (minmax.second - minmax.first) + minmax.first;
    return value;
}

std::pair<double, double> pdal::e57plugin::getLimits(const e57::StructureNode &prototype, const std::string &fieldName)
{
    double max = std::nan("max");
    double min = std::nan("min");
    assert(max != max && min != min);

    std::string minKey = fieldName + "Minimum";
    std::string maxKey = fieldName + "Maximum";
    std::string boundingBoxName = fieldName+"Limits";

    if (fieldName.substr(0,5) == "color")
        boundingBoxName = "colorLimits";
    else if (fieldName[0] == 'x' || fieldName[0] == 'y' || fieldName[0] == 'z')
        boundingBoxName = "cartesianBounds";

    if ( prototype.isDefined(boundingBoxName) )
	{
		e57::StructureNode intbox(prototype.get(boundingBoxName));
		if ( intbox.get(maxKey).type() == e57::E57_SCALED_INTEGER )
		{
			max = static_cast<e57::ScaledIntegerNode>(intbox.get(maxKey)).scaledValue();
			min = static_cast<e57::ScaledIntegerNode>(intbox.get(minKey)).scaledValue();
		}
		else if ( intbox.get(maxKey).type() == e57::E57_FLOAT )
		{
			max  = static_cast<e57::FloatNode>(intbox.get(maxKey)).value();
			min  = static_cast<e57::FloatNode>(intbox.get(minKey)).value();
		}
		else if ( intbox.get(maxKey).type() == e57::E57_INTEGER)
		{
			max  = static_cast<double>(static_cast<e57::IntegerNode>(intbox.get(maxKey)).value());
			min = static_cast<double>(static_cast<e57::IntegerNode>(intbox.get(minKey)).value());
		}
	}
    else if ( prototype.isDefined(fieldName) )
	{
		if (prototype.get(fieldName).type() == e57::E57_INTEGER)
		{
           min = static_cast<double>(static_cast<e57::IntegerNode>(prototype.get(fieldName)).minimum());
            max = static_cast<double>(static_cast<e57::IntegerNode>(prototype.get(fieldName)).maximum());
		}
		else if (prototype.get(fieldName).type() == e57::E57_SCALED_INTEGER)
		{
			double scale = static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName)).scale();
			double offset = static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName)).offset();
            int64_t minimum = static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName)).minimum();
            int64_t maximum = static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName)).maximum();
            min = minimum * scale + offset;	
            max = maximum * scale + offset;
		}
		else if (prototype.get(fieldName).type() == e57::E57_FLOAT)
		{
            min  = static_cast<e57::FloatNode>(prototype.get(fieldName)).minimum();
            max = static_cast<e57::FloatNode>(prototype.get(fieldName)).maximum();  
		}
	}
    return {min,max};
}

std::pair<double,double> pdal::e57plugin::getPdalBounds(pdal::Dimension::Id id)
{
    using Dim = pdal::Dimension::Id;
    switch (id)
    {
        case Dim::Red:
            return {std::numeric_limits<uint16_t>::min(),std::numeric_limits<uint16_t>::max()};    
        case Dim::Blue:
            return {std::numeric_limits<uint16_t>::min(),std::numeric_limits<uint16_t>::max()};
        case Dim::Green:
            return {std::numeric_limits<uint16_t>::min(),std::numeric_limits<uint16_t>::max()};
        case Dim::Intensity:
            return {std::numeric_limits<uint16_t>::min(),std::numeric_limits<uint16_t>::max()};
        default:
            std::string msg ="Dimension " + pdal::Dimension::name(id) + " is not currently supported.";
            throw std::invalid_argument(msg);
    }
}

