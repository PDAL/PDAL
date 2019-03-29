#include <limits>

#include "utils.hpp"

pdal::Dimension::Id pdal::e57plugin::e57ToPdal(std::string e57Dimension) {
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
    else
    {
        return pdal::Dimension::Id::Unknown;
    }
}

std::string pdal::e57plugin::pdalToE57(pdal::Dimension::Id pdalDimension)
{
    if (pdalDimension == pdal::Dimension::Id::X)
    {
        return  "cartesianX";
    }
    else if (pdalDimension == pdal::Dimension::Id::Y)
    {
        return  "cartesianY";
    }
    else if (pdalDimension == pdal::Dimension::Id::Z)
    {
        return  "cartesianZ";
    }
    else if (pdalDimension == pdal::Dimension::Id::Red)
    {
        return  "colorRed";
    }
    else if (pdalDimension == pdal::Dimension::Id::Blue)
    {
        return  "colorBlue";
    }
    else if (pdalDimension == pdal::Dimension::Id::Green)
    {
        return  "colorGreen";
    }
    else if (pdalDimension == pdal::Dimension::Id::Intensity)
    {
        return  "intensity";
    }
    return std::string();
}

std::vector<pdal::Dimension::Id> pdal::e57plugin::supportedPdalTypes()
{
    return {pdal::Dimension::Id::X,pdal::Dimension::Id::Y,pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Red,pdal::Dimension::Id::Green,pdal::Dimension::Id::Blue,pdal::Dimension::Id::Intensity};
}

std::vector<std::string> pdal::e57plugin::supportedE57Types()
{
    return {"cartesianX","cartesianY","cartesianZ","colorRed","colorGreen","colorBlue","intensity"};
}

double pdal::e57plugin::rescaleE57ToPdalValue(std::string e57Dimension, double value, const std::pair<double,double>& e57Bounds)
{
    assert(e57Bounds.second >= e57Bounds.first);
    if (e57Bounds.second == e57Bounds.first)
    {
        return value;
    }
    
    auto pdalDimension = pdal::e57plugin::e57ToPdal(e57Dimension);
    std::pair<double,double> minmax;
    try
    {
        minmax = getPdalBounds(pdalDimension);
    }
    catch (std::invalid_argument e)
    {
        return value;
    }
    value = (value - e57Bounds.first) / (e57Bounds.second - e57Bounds.first + 1e-10);
    value = value * (minmax.second - minmax.first) + minmax.first;
    return value;
}

std::pair<double,double> pdal::e57plugin::getLimits(const e57::StructureNode &prototype, std::string fieldName)
{
    double max = std::nan("max");
    double min = std::nan("min");
    assert(max != max && min != min);

    std::string minKey = fieldName + "Minimum";
    std::string maxKey = fieldName + "Maximum";
    std::string boundingBoxName = fieldName+"Limits";

    if (fieldName.substr(0,5) == "color")
    {
        boundingBoxName = "colorLimits";
    }


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
	
	if (max == 0. && prototype.isDefined(fieldName) )
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

std::string pdal::e57plugin::getDescription(const e57::Node &node)
{
	std::string infoStr = node.elementName() + " - ";
//	switch(node.type())
//	{
//	case e57::E57_STRUCTURE:
//		{
//			e57::StructureNode s = static_cast<e57::StructureNode>(node);
//			infoStr += fmt::format("STRUCTURE, %d child(ren)",s.childCount());
//		}
//		break;
//	case e57::E57_VECTOR:
//		{
//			e57::VectorNode v = static_cast<e57::VectorNode>(node);
//			infoStr += fmt::format("VECTOR, %d child(ren)",v.childCount());
//		}
//		break;
//	case e57::E57_COMPRESSED_VECTOR:
//		{
//			e57::CompressedVectorNode cv = static_cast<e57::CompressedVectorNode>(node);
//			infoStr += fmt::format("COMPRESSED VECTOR, %d elements",cv.childCount());
//		}
//		break;
//	case e57::E57_INTEGER:
//		{
//			e57::IntegerNode i = static_cast<e57::IntegerNode>(node);
//			infoStr += fmt::format("%d (INTEGER)",i.value());
//		}
//		break;
//	case e57::E57_SCALED_INTEGER:
//		{
//			e57::ScaledIntegerNode si = static_cast<e57::ScaledIntegerNode>(node);
//			infoStr += fmt::format("%d (SCALED INTEGER)",si.scaledValue());
//		}
//		break;
//	case e57::E57_FLOAT:
//		{
//			e57::FloatNode f = static_cast<e57::FloatNode>(node);
//			infoStr += fmt::format("%.2f (FLOAT)",f.value());
//		}
//		break;
//	case e57::E57_STRING:
//		{
//			e57::StringNode s = static_cast<e57::StringNode>(node);
//			infoStr += s.value();
//		}
//		break;
//	case e57::E57_BLOB:
//		{
//			e57::BlobNode b = static_cast<e57::BlobNode>(node);
//			infoStr += fmt::format("BLOB, size=%d",b.byteCount());
//		}
//		break;
//	default:
//		{
//			infoStr += "INVALID";
//		}
//		break;
//	}
	return infoStr;
}

e57::Node pdal::e57plugin::getNode(const e57::Node &parent, std::string childName)
{
	if (parent.type() == e57::E57_STRUCTURE)
	{
		e57::StructureNode s = static_cast<e57::StructureNode>(parent);
		if (!s.isDefined(childName))
		{
			std::string message =
				"Node " +  parent.elementName() + " does not have a children named " + childName;
			throw std::invalid_argument(message);
		}
		else
		{
			return s.get(childName);
		}
	}
	else if (parent.type() == e57::E57_VECTOR)
	{
		e57::VectorNode v = static_cast<e57::VectorNode>(parent);
		if (!v.isDefined(childName))
		{
			std::string message =
                    "Node " +  parent.elementName() + " does not have a children named " + childName;
			throw std::invalid_argument(message);
		}
		else
		{
			return v.get(childName);
		}
	}
else
	{
		std::string message =
			"Node " + parent.elementName() + " is not hierarchical and does not have children";
		throw std::invalid_argument(message);
	}
}

