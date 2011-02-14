/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Schema implementation for C++ libLAS
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <libpc/Schema.hpp>
#include <libpc/Utils.hpp>

// boost
#include <boost/concept_check.hpp>
#include <boost/foreach.hpp>

using namespace boost;

namespace libpc
{


Schema::Schema()
    : m_byteSize(0)
{
    return;
}


/// copy constructor
Schema::Schema(Schema const& other) 
    : m_dimensions(other.m_dimensions)
    , m_byteSize(other.m_byteSize)
{
}


// assignment constructor
Schema& Schema::operator=(Schema const& rhs)
{
    if (&rhs != this)
    {
        m_dimensions = rhs.m_dimensions;
        m_byteSize = rhs.m_byteSize;
    }

    return *this;
}


property_tree::ptree Schema::getPTree() const
{
    using property_tree::ptree;
    ptree pt;

    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        pt.add_child("LASSchema.dimensions.dimension", (*iter).GetPTree());
    }

    return pt;
}


void Schema::calculateSizes()
{
    // to make life easy, for now we are going to assume that each Dimension 
    // is byte-aligned and occupies an integral number of bytes

    std::size_t offset = 0;

    for (DimensionsIter iter = m_dimensions.begin(); iter != m_dimensions.end(); ++iter)
    {
        Dimension& t = *iter;

        t.setByteOffset(offset);

        offset += t.getByteSize();
    }

    m_byteSize = offset;

    return;
}


std::size_t Schema::getByteSize() const
{
    return m_byteSize;
}


void Schema::addDimension(Dimension const& dim)
{
    // BUG: assert not already added

    m_dimensions.push_back(dim);

    // Update all of our sizes
    calculateSizes();

    return;
}


std::vector<std::string> Schema::getDimensionNames() const
{
    std::vector<std::string> output;

    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        output.push_back(iter->getName());
    }

    return output;
}



std::ostream& operator<<(std::ostream& os, Schema const& schema)
{
    using property_tree::ptree;
    ptree tree = schema.getPTree();

    os << "---------------------------------------------------------" << std::endl;
    os << "  Schema Summary" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    ptree::const_iterator i;

    ptree dims = tree.get_child("LASSchema.dimensions");
    ///////////os << "  Point Format ID:             " << tree.get<std::string>("LASSchema.formatid") << std::endl;
    os << "  Number of dimensions:        " << dims.size() << std::endl;
    os << "  Size in bytes:               " << schema.getByteSize() << std::endl;

    os << std::endl;
    os << "  Dimensions" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    os << "  ";

    const Schema::Dimensions& dimensions = schema.getDimensions();
    for (Schema::DimensionsCIter iter = dimensions.cbegin(); iter != dimensions.cend(); ++iter)
    {
        os << *iter;
        os << "  ";
    }


    os << std::endl;

    return os;
}


LasSchema::LasSchema(PointFormatName data_format_id)
    : Schema()
    , m_data_format_id(data_format_id)
{
    update_required_dimensions(data_format_id);
}


/// copy constructor
LasSchema::LasSchema(LasSchema const& other)
    : Schema(other)
    , m_data_format_id(other.m_data_format_id)
{
}


// assignment constructor
LasSchema& LasSchema::operator=(LasSchema const& rhs)
{
    if (&rhs != this)
    {
        *((Schema*)this) = (Schema const&)rhs;
        m_data_format_id = rhs.m_data_format_id;
    }

    return *this;
}


void LasSchema::add_record0_dimensions()
{
    std::ostringstream text;

    Dimension x("X", Dimension::float_t);
    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    addDimension(x);
    text.str("");

    Dimension y("Y", Dimension::float_t);
    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    addDimension(y);
    text.str("");

    Dimension z("Z", Dimension::float_t);
    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    addDimension(z);
    text.str("");

    Dimension intensity("Intensity", Dimension::int16_t);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    addDimension(intensity);
    text.str("");

    Dimension return_no("Return Number", Dimension::bits_t, 3);
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    addDimension(return_no);
    text.str("");

    Dimension no_returns("Number of Returns", Dimension::bits_t, 3);
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    addDimension(no_returns);
    text.str("");

    Dimension scan_dir("Scan Direction", Dimension::bits_t, 1);
    text << "The Scan Direction Flag denotes the direction at which the "
         "scanner mirror was traveling at the time of the output pulse. "
         "A bit value of 1 is a positive scan direction, and a bit value "
         "of 0 is a negative scan direction (where positive scan direction "
         "is a scan moving from the left side of the in-track direction to "
         "the right side and negative the opposite). ";
    scan_dir.setDescription(text.str());
    addDimension(scan_dir);
    text.str("");

    Dimension edge("Flightline Edge", Dimension::bits_t, 1);
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    addDimension(edge);
    text.str("");

    Dimension classification("Classification", Dimension::uint8_t);
    text << "Classification in LAS 1.0 was essentially user defined and optional. "
         "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
         "the field is now mandatory. If a point has never been classified, this "
         "byte must be set to zero. There are no user defined classes since "
         "both point format 0 and point format 1 supply 8 bits per point for "
         "user defined operations. Note that the format for classification is a "
         "bit encoded field with the lower five bits used for class and the "
         "three high bits used for flags.";
    classification.setDescription(text.str());
    addDimension(classification);
    text.str("");

    Dimension scan_angle("Scan Angle Rank", Dimension::int8_t);
    text << "The Scan Angle Rank is a signed one-byte number with a "
         "valid range from -90 to +90. The Scan Angle Rank is the "
         "angle (rounded to the nearest integer in the absolute "
         "value sense) at which the laser point was output from the "
         "laser system including the roll of the aircraft. The scan "
         "angle is within 1 degree of accuracy from +90 to –90 degrees. "
         "The scan angle is an angle based on 0 degrees being nadir, "
         "and –90 degrees to the left side of the aircraft in the "
         "direction of flight.";
    scan_angle.setDescription(text.str());
    addDimension(scan_angle);
    text.str("");

    Dimension user_data("User Data", Dimension::uint8_t);
    text << "This field may be used at the user’s discretion";
    user_data.setDescription(text.str());
    addDimension(user_data);
    text.str("");

    Dimension point_source_id("Point Source ID", Dimension::uint16_t);
    text << "This value indicates the file from which this point originated. "
         "Valid values for this field are 1 to 65,535 inclusive with zero "
         "being used for a special case discussed below. The numerical value "
         "corresponds to the File Source ID from which this point originated. "
         "Zero is reserved as a convenience to system implementers. A Point "
         "Source ID of zero implies that this point originated in this file. "
         "This implies that processing software should set the Point Source "
         "ID equal to the File Source ID of the file containing this point "
         "at some time during processing. ";
    point_source_id.setDescription(text.str());

    addDimension(point_source_id);
    text.str("");

    return;
}

void LasSchema::add_color()
{
    std::ostringstream text;

    Dimension red("Red", Dimension::uint16_t);
    text << "The red image channel value associated with this point";
    red.setDescription(text.str());
    addDimension(red);
    text.str("");

    Dimension green("Green", Dimension::uint16_t);
    text << "The green image channel value associated with this point";
    green.setDescription(text.str());
    addDimension(green);
    text.str("");

    Dimension blue("Blue", Dimension::uint16_t);
    text << "The blue image channel value associated with this point";
    blue.setDescription(text.str());
    addDimension(blue);
    text.str("");

}

void LasSchema::add_time()
{
    std::ostringstream text;

    Dimension t("Time", Dimension::uint64_t);
    text << "The GPS Time is the double floating point time tag value at "
         "which the point was acquired. It is GPS Week Time if the "
         "Global Encoding low bit is clear and Adjusted Standard GPS "
         "Time if the Global Encoding low bit is set (see Global Encoding "
         "in the Public Header Block description).";
    t.setDescription(text.str());
    addDimension(t);
    text.str("");

    return;
}

void LasSchema::update_required_dimensions(PointFormatName data_format_id)
{
    // Add the base dimensions
    add_record0_dimensions();

    switch (data_format_id)
    {
    case ePointFormat3:
        add_time();
        add_color();
        break;
    case ePointFormat2:
        add_color();
        break;
    case ePointFormat1:
        add_time();
        break;
    case ePointFormat0:
        break;

    default:
        std::ostringstream oss;
        oss << "Unhandled PointFormatName id " << static_cast<boost::uint32_t>(data_format_id);
        throw std::runtime_error(oss.str());
    }

    calculateSizes();

    return;
}


} // namespace libpc
