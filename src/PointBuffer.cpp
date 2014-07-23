/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <iomanip>

#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>

#include <boost/lexical_cast.hpp>

namespace pdal
{

pdal::Bounds<double> PointBuffer::calculateBounds(bool is3d) const
{
    pdal::Schema const& schema = getSchema();

    pdal::Bounds<double> output;

    DimensionPtr dimX = schema.getDimension("X");
    DimensionPtr dimY = schema.getDimension("Y");
    DimensionPtr dimZ = schema.getDimension("Z");

    Vector<double> v;

    bool first = true;
    for (PointId idx = 0; idx < size(); idx++)
    {
        double x = getFieldAs<double>(dimX, idx);
        double y = getFieldAs<double>(dimY, idx);
        double z = getFieldAs<double>(dimZ, idx);

        if (is3d)
        {
            if (first)
            {
                output = pdal::Bounds<double>(x, y, z, x, y, z);
                first = false;
                v.add(x);
                v.add(y);
                v.add(z);
            }
            v[0] = x;
            v[1] = y;
            v[2] = z;
            output.grow(v);
        }
        else
        {
            if (first)
            {
                output = pdal::Bounds<double>(x, y, x, y);
                first = false;
                v.add(x);
                v.add(y);
            }
            v[0] = x;
            v[1] = y;
            output.grow(v);
        }
    }
    return output;
}


boost::property_tree::ptree PointBuffer::toPTree() const
{
    boost::property_tree::ptree tree;

    DimensionList dims = getSchema().getDimensions();

    for (PointId idx = 0; idx < size(); idx++)
    {
        std::string pointstring = boost::lexical_cast<std::string>(idx) + ".";

        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimensionPtr d = *di;
            std::string key = pointstring + d->getName();
            double v = getFieldAs<double>(d, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            tree.add(key, value);
        }
    }
    return tree;
}


std::ostream& PointBuffer::toRST(std::ostream& os) const
{
    DimensionList dims = getSchema().getDimensions();

    uint32_t ns_column(32);
    uint32_t name_column(20);
    uint32_t value_column(40);
    
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        DimensionPtr d = *di;
        name_column = std::max(name_column, (uint32_t)d->getName().size());
        ns_column = std::max(ns_column, (uint32_t)d->getNamespace().size());
    }
    
    std::ostringstream thdr;
    for (unsigned i = 0; i < name_column-1; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < value_column-1; ++i)
        thdr << "=";        
    thdr << " ";
    for (unsigned i = 0; i < ns_column-1; ++i)
        thdr << "=";    
    thdr << " ";

    name_column--;
    unsigned step_back(3);

    std::string hdr(80, '-');
    for (PointId idx = 0; idx < size(); ++idx)
    {
        os << "Point " << idx << std::endl;
        os << hdr << std::endl << std::endl;
        os << thdr.str() << std::endl;
        os << std::setw(name_column-step_back) << "Name" <<
            std::setw(value_column-step_back) << "Value"  <<
            std::setw(ns_column-step_back) << "Namespace" << std::endl;
        os << thdr.str() << std::endl;        
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimensionPtr d = *di;
            double v = getFieldAs<double>(d, idx);
            std::string value = boost::lexical_cast<std::string>(v);
            os << std::left << std::setw(name_column) << d->getName() <<
                std::right << std::setw(value_column) << value <<
                std::setw(ns_column) << d->getNamespace() << std::endl;
        }
        os << thdr.str() << std::endl << std::endl;
    }
    os << std::endl << std::endl;
    return os;
}


std::ostream& operator<<(std::ostream& ostr, const PointBuffer& buf)
{
    using std::endl;

    DimensionList dims = buf.getSchema().getDimensions();

    point_count_t numPoints = buf.size();
    ostr << "Contains " << numPoints << "  points" << endl;
    for (PointId idx = 0; idx < numPoints; idx++)
    {
        ostr << "Point: " << idx << endl;

        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimensionPtr d = *di;

            ostr << d->getName() << " (" << d->getInterpretationName() <<
                ") : ";

            switch (d->getInterpretation())
            {
                case dimension::SignedInteger:
                    if (d->getByteSize() == 1)
                        ostr << (int)(buf.getField<int8_t>(d, idx));
                    if (d->getByteSize() == 2)
                        ostr << buf.getField<int16_t>(d, idx);
                    if (d->getByteSize() == 4)
                        ostr << buf.getField<int32_t>(d, idx);
                    if (d->getByteSize() == 8)
                        ostr << buf.getField<int64_t>(d, idx);
                    break;
                case dimension::UnsignedInteger:
                    if (d->getByteSize() == 1)
                        ostr << (unsigned)(buf.getField<uint8_t>(d, idx));
                    if (d->getByteSize() == 2)
                        ostr << buf.getField<uint16_t>(d, idx);
                    if (d->getByteSize() == 4)
                        ostr << buf.getField<uint32_t>(d, idx);
                    if (d->getByteSize() == 8)
                        ostr << buf.getField<uint64_t>(d, idx);
                    break;
                case dimension::Float:
                    if (d->getByteSize() == 4)
                        ostr << buf.getField<float>(d, idx);
                    if (d->getByteSize() == 8)
                        ostr << buf.getField<double>(d, idx);
                    break;
                default:
                    throw;
            }

            ostr << endl;
        }
    }
    return ostr;
}

} // namespace pdal

