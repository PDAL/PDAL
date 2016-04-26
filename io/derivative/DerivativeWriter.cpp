/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers, brad.chambers@gmail.com
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

#include "DerivativeWriter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_macros.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>

#include <boost/filesystem.hpp>

#include "gdal_priv.h" // For File I/O
#include "gdal_version.h" // For version info
#include "ogr_spatialref.h"  //For Geographic Information/Transformations

namespace pdal
{
static PluginInfo const s_info = PluginInfo(
                                     "writers.derivative",
                                     "Derivative writer",
                                     "http://pdal.io/stages/writers.derivative.html");

CREATE_STATIC_PLUGIN(1, 0, DerivativeWriter, Writer, s_info)

std::string DerivativeWriter::getName() const
{
    return s_info.name;
}

const double c_pi = 3.14159265358979323846; /*!< PI value */
const float c_background = FLT_MIN;

DerivativeWriter::DerivativeWriter()
{
    GDALAllRegister();
}


void DerivativeWriter::processOptions(const Options& ops)
{
    m_GRID_DIST_X = ops.getValueOrDefault<double>("grid_dist_x", 15.0);
    m_GRID_DIST_Y = ops.getValueOrDefault<double>("grid_dist_y", 15.0);
    handleFilenameTemplate();

    std::map<std::string, PrimitiveType> primtypes;
    primtypes["slope_d8"] = SLOPE_D8;
    primtypes["slope_fd"] = SLOPE_FD;
    primtypes["aspect_d8"] = ASPECT_D8;
    primtypes["aspect_fd"] = ASPECT_FD;
    primtypes["hillshade"] = HILLSHADE;
    primtypes["contour_curvature"] = CONTOUR_CURVATURE;
    primtypes["profile_curvature"] = PROFILE_CURVATURE;
    primtypes["tangential_curvature"] = TANGENTIAL_CURVATURE;
    primtypes["total_curvature"] = TOTAL_CURVATURE;
    primtypes["catchment_area"] = CATCHMENT_AREA;

    std::string primTypes = ops.getValueOrDefault("primitive_type", "slope_d8");
    StringList types = Utils::split2(primTypes, ',');

    if (m_hashPos == std::string::npos && types.size() > 1)
    {
        std::ostringstream oss;

        oss << getName() << ": No template placeholder ('#') found in "
            "filename '" << m_filename << "' when one is required with "
            "multiple primitive types.";
        throw pdal_error(oss.str());
    }

    for (std::string os : types)
    {
        Utils::trim(os);
        std::string s = Utils::tolower(os);
        auto pi = primtypes.find(s);
        if (pi == primtypes.end())
        {
            std::ostringstream oss;
            oss << getName() << ": Unrecognized primitive type '" << os <<
                "'.";
            throw pdal_error(oss.str());
        }
        TypeOutput to;
        to.m_type = pi->second;
        to.m_filename = generateFilename(pi->first) ;
        m_primitiveTypes.push_back(to);
    }
}


std::string
DerivativeWriter::generateFilename(const std::string& primName) const
{
    // We've already checked during argument parsing that we have a valid
    // template placeholder (#) if necessary.
    std::string filename = m_filename;
    if (m_hashPos != std::string::npos)
        filename.replace(m_hashPos, 1, primName);
    return filename;
}


void DerivativeWriter::initialize()
{

    setBounds(BOX2D());
}


Options DerivativeWriter::getDefaultOptions()
{
    Options options;

    options.add("grid_dist_x", 15.0, "X grid distance");
    options.add("grid_dist_y", 15.0, "Y grid distance");
    options.add("primitive_type", "slope_d8", "Primitive type");

    return options;
}

double DerivativeWriter::GetNeighbor(Eigen::MatrixXd* data, int row, int col,
    Direction d)
{
    double val;
    switch (d)
    {
        case NORTH:
            val = ((*data)(row-1, col));
            break;
        case SOUTH:
            val = ((*data)(row+1, col));
            break;
        case EAST:
            val = ((*data)(row, col+1));
            break;
        case WEST:
            val = ((*data)(row, col-1));
            break;
        case NORTHEAST:
            val = ((*data)(row-1, col+1));
            break;
        case NORTHWEST:
            val = ((*data)(row-1, col-1));
            break;
        case SOUTHEAST:
            val = ((*data)(row+1, col+1));
            break;
        case SOUTHWEST:
            val = ((*data)(row+1, col-1));
            break;
        default:
            val = ((*data)(row, col));
            break;
    }
    return val;
}


double DerivativeWriter::determineSlopeFD(Eigen::MatrixXd* data, int row,
        int col, double postSpacing, double valueToIgnore)
{
    double tSlopeVal = valueToIgnore;
    double tSlopeValDegree = valueToIgnore;

    double mean = 0.0;
    unsigned int nvals = 0;

    double val = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(val);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);

    mean /= nvals;

    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;

    double zX = (east - west) / (2 * postSpacing);
    double zY = (north - south) / (2 * postSpacing);
    double p = (zX * zX) + (zY * zY);

    tSlopeVal = std::sqrt(p);

    if (tSlopeVal != valueToIgnore)
        tSlopeValDegree = atan(tSlopeVal) * (180.0f / c_pi);

    return tSlopeValDegree;
}


double DerivativeWriter::determineSlopeD8(Eigen::MatrixXd* data, int row,
        int col, double postSpacing, double valueToIgnore)
{
    double tPhi1 = 1.0f;
    double tPhi2 = sqrt(2.0f);
    double tSlopeVal = valueToIgnore;
    double tSlopeValDegree = valueToIgnore;

    double val = static_cast<double>((*data)(row, col));
    if (val == valueToIgnore)
        return val;

    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);
    double northeast = GetNeighbor(data, row, col, NORTHEAST);
    double northwest = GetNeighbor(data, row, col, NORTHWEST);
    double southeast = GetNeighbor(data, row, col, SOUTHEAST);
    double southwest = GetNeighbor(data, row, col, SOUTHWEST);

    auto checkVal = [val, &tSlopeVal, valueToIgnore, postSpacing]
        (double neighbor, double phi)
    {
        if (neighbor != valueToIgnore)
        {
            neighbor = (val - neighbor) / (postSpacing * phi);
            if ((std::fabs(neighbor) > std::fabs(tSlopeVal)) ||
                (tSlopeVal == valueToIgnore))
                tSlopeVal = neighbor;
        }
    };

    checkVal(north, tPhi1);
    checkVal(south, tPhi1);
    checkVal(east, tPhi1);
    checkVal(west, tPhi1);
    checkVal(northeast, tPhi2);
    checkVal(northwest, tPhi2);
    checkVal(southeast, tPhi2);
    checkVal(southwest, tPhi2);

    if (tSlopeVal != valueToIgnore)
        tSlopeValDegree = atan(tSlopeVal) * (180.0f / c_pi);

    return tSlopeValDegree;
}


double DerivativeWriter::determineAspectFD(Eigen::MatrixXd* data, int row,
        int col, double postSpacing, double valueToIgnore)
{
    double mean = 0.0;
    unsigned int nvals = 0;

    double val = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(val);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);

    mean /= nvals;

    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;

    double zX = (east - west) / (2 * postSpacing);
    double zY = (north - south) / (2 * postSpacing);
    double p = (zX * zX) + (zY * zY);

    return 180.0 - std::atan(zY/zX) + 90.0 * (zX / std::fabs(zX));
}


double DerivativeWriter::determineAspectD8(Eigen::MatrixXd* data, int row,
        int col, double postSpacing)
{
    double tPhi1 = 1.0f;
    double tPhi2 = sqrt(2.0f);
    double tH = postSpacing;
    double tVal, tN, tS, tE, tW, tNW, tNE, tSW, tSE, nextTVal;
    double tSlopeVal = std::numeric_limits<double>::max(), tSlopeValDegree;
//     int tNextY, tNextX;
    unsigned int j = 0;

    tVal = (*data)(row, col);
    if (tVal == std::numeric_limits<double>::max())
        return tVal;

    //North
    nextTVal = GetNeighbor(data, row, col, NORTH);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tN = (tVal - nextTVal) / (tH * tPhi1);
        if (tN > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tN;
            j = 8;
        }
    }
    //South
    nextTVal = GetNeighbor(data, row, col, SOUTH);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tS = (tVal - nextTVal) / (tH * tPhi1);
        if (tS > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tS;
            j = 4;
        }
    }
    //East
    nextTVal = GetNeighbor(data, row, col, EAST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tE = (tVal - nextTVal) / (tH * tPhi1);
        if (tE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tE;
            j = 2;
        }
    }
    //West
    nextTVal = GetNeighbor(data, row, col, WEST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tW = (tVal - nextTVal) / (tH * tPhi1);
        if (tW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tW;
            j = 6;
        }
    }
    //NorthEast
    nextTVal = GetNeighbor(data, row, col, NORTHEAST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tNE = (tVal - nextTVal) / (tH * tPhi2);
        if (tNE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tNE;
            j = 1;
        }
    }
    //NorthWest
    nextTVal = GetNeighbor(data, row, col, NORTHWEST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tNW = (tVal - nextTVal) / (tH * tPhi2);
        if (tNW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tNW;
            j = 7;
        }
    }
    //SouthEast
    nextTVal = GetNeighbor(data, row, col, SOUTHEAST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tSE = (tVal - nextTVal) / (tH * tPhi2);
        if (tSE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tSE;
            j = 3;
        }
    }
    //SouthWest
    nextTVal = GetNeighbor(data, row, col, SOUTHWEST);
    if (nextTVal < std::numeric_limits<double>::max())
    {
        tSW = (tVal - nextTVal) / (tH * tPhi2);
        if (tSW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        {
            tSlopeVal = tSW;
            j = 5;
        }
    }

    //tSlopeValDegree = 45 * j;
    tSlopeValDegree = std::pow(2.0,j-1);

    return tSlopeValDegree;
}

int DerivativeWriter::determineCatchmentAreaD8(Eigen::MatrixXd* data,
        Eigen::MatrixXd* area, int row, int col, double postSpacing)
{
    if ((*area)(row, col) > 0)
    {
        return (*area)(row, col);
    }
    else
    {
        (*area)(row, col) = 1;

        for (int i = 1; i < 9; ++i)
        {
            int j, k;
            switch (i)
            {
                case 1:
                    j = row - 1;
                    k = col + 1;
                    break;

                case 2:
                    j = row;
                    k = col + 1;
                    break;

                case 3:
                    j = row + 1;
                    k = col + 1;
                    break;

                case 4:
                    j = row + 1;
                    k = col;
                    break;

                case 5:
                    j = row + 1;
                    k = col - 1;
                    break;

                case 6:
                    j = row;
                    k = col - 1;
                    break;

                case 7:
                    j = row - 1;
                    k = col - 1;
                    break;

                case 8:
                    j = row - 1;
                    k = col;
                    break;
            }

            if ((*area)(j, k) > 0)
                (*area)(row, col) += determineCatchmentAreaD8(data, area, j, k,
                                     postSpacing);

            // not quite complete here...
        }

        //double tPhi1 = 1.0f;
        //double tPhi2 = sqrt(2.0f);
        //double tH = postSpacing;
        //double tVal, tN, tS, tE, tW, tNW, tNE, tSW, tSE, nextTVal;
        //double tSlopeVal = std::numeric_limits<double>::max(), tSlopeValDegree;
        //int tNextY, tNextX;
        //unsigned int j = 0;

        //tVal = (*data)(row, col);
        //if (tVal == std::numeric_limits<double>::max())
        //  return tVal;

        ////North
        //tNextY = row - 1;
        //tNextX = col;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tN = (tVal - nextTVal) / (tH * tPhi1);
        //  if (tN > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tN;
        //    j = 8;
        //  }
        //}
        ////South
        //tNextY = row + 1;
        //tNextX = col;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tS = (tVal - nextTVal) / (tH * tPhi1);
        //  if (tS > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tS;
        //    j = 4;
        //  }
        //}
        ////East
        //tNextY = row;
        //tNextX = col + 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tE = (tVal - nextTVal) / (tH * tPhi1);
        //  if (tE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tE;
        //    j = 2;
        //  }
        //}
        ////West
        //tNextY = row;
        //tNextX = col - 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tW = (tVal - nextTVal) / (tH * tPhi1);
        //  if (tW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tW;
        //    j = 6;
        //  }
        //}
        ////NorthEast
        //tNextY = row - 1;
        //tNextX = col + 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tNE = (tVal - nextTVal) / (tH * tPhi2);
        //  if (tNE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tNE;
        //    j = 1;
        //  }
        //}
        ////NorthWest
        //tNextY = row - 1;
        //tNextX = col - 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tNW = (tVal - nextTVal) / (tH * tPhi2);
        //  if (tNW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tNW;
        //    j = 7;
        //  }
        //}
        ////SouthEast
        //tNextY = row + 1;
        //tNextX = col + 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tSE = (tVal - nextTVal) / (tH * tPhi2);
        //  if (tSE > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tSE;
        //    j = 3;
        //  }
        //}
        ////SouthWest
        //tNextY = row + 1;
        //tNextX = col - 1;
        //nextTVal = (*data)(tNextY, tNextX);
        //if (nextTVal < std::numeric_limits<double>::max())
        //{
        //  tSW = (tVal - nextTVal) / (tH * tPhi2);
        //  if (tSW > tSlopeVal || tSlopeVal == std::numeric_limits<double>::max())
        //  {
        //    tSlopeVal = tSW;
        //    j = 5;
        //  }
        //}

        //switch (j)
        //{
        //case 1:
        //  tNextY = row - 1;
        //  tNextX = col + 1;
        //  break;

        //case 2:
        //  tNextY = row;
        //  tNextX = col + 1;
        //  break;

        //case 3:
        //  tNextY = row + 1;
        //  tNextX = col + 1;
        //  break;

        //case 4:
        //  tNextY = row + 1;
        //  tNextX = col;
        //  break;

        //case 5:
        //  tNextY = row + 1;
        //  tNextX = col - 1;
        //  break;

        //case 6:
        //  tNextY = row;
        //  tNextX = col - 1;
        //  break;

        //case 7:
        //  tNextY = row - 1;
        //  tNextX = col - 1;
        //  break;

        //case 8:
        //  tNextY = row - 1;
        //  tNextX = col;
        //  break;
        //}
        //(*area)(row, col) = determineCatchmentAreaD8(data, area, tNextY, tNextX, postSpacing);
    }
    return 0;
}

double DerivativeWriter::determineHillshade(Eigen::MatrixXd* data, int row,
        int col, double zenithRad, double azimuthRad, double postSpacing)
{
    //ABELL - tEVar not currently used.
    //double tAVar, tBVar, tCVar, tDVar, tEVar, tFVar, tGVar, tHVar, tIVar;
    double tAVar, tBVar, tCVar, tDVar, tFVar, tGVar, tHVar, tIVar;
    double tDZDX, tDZDY, tSlopeRad, tAspectRad = 0.0;
    double tHillShade;

    tAVar = GetNeighbor(data, row, col, NORTHWEST);
    tBVar = GetNeighbor(data, row, col, NORTH);
    tCVar = GetNeighbor(data, row, col, NORTHWEST);
    tDVar = GetNeighbor(data, row, col, WEST);
    //tEVar = (double)(*data)(row, col);
    tFVar = GetNeighbor(data, row, col, EAST);
    tGVar = GetNeighbor(data, row, col, SOUTHWEST);
    tHVar = GetNeighbor(data, row, col, SOUTH);
    tIVar = GetNeighbor(data, row, col, SOUTHEAST);

    tDZDX = ((tCVar + 2 * tFVar + tIVar) - (tAVar + 2 * tDVar + tGVar)) /
            (8 * postSpacing);
    tDZDY = ((tGVar + 2* tHVar + tIVar) - (tAVar + 2 * tBVar + tCVar))  /
            (8 * postSpacing);
    tSlopeRad = atan(sqrt(pow(tDZDX, 2) + pow(tDZDY, 2)));

    if (tDZDX == 0)
    {
        if (tDZDY > 0)
        {
            tAspectRad = c_pi / 2;
        }
        else if (tDZDY < 0)
        {
            tAspectRad = (2 * c_pi) - (c_pi / 2);
        }
        else
        {
            //ABELL - This looks wrong.  At least needs a comment.
            // tAspectRad = tAspectRad;
            ;
        }
    }
    else
    {
        tAspectRad = atan2(tDZDY, -1 * tDZDX);
        if (tAspectRad < 0)
        {
            tAspectRad = 2 * c_pi + tAspectRad;
        }
    }

    tHillShade = (((cos(zenithRad) * cos(tSlopeRad)) + (sin(zenithRad) *
                   sin(tSlopeRad) * cos(azimuthRad - tAspectRad))));

    return tHillShade;
}


double DerivativeWriter::determineContourCurvature(Eigen::MatrixXd* data,
        int row, int col, double postSpacing, double valueToIgnore)
{
    double mean = 0.0;
    unsigned int nvals = 0;

    double value = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);
    double northeast = GetNeighbor(data, row, col, NORTHEAST);
    double northwest = GetNeighbor(data, row, col, NORTHWEST);
    double southeast = GetNeighbor(data, row, col, SOUTHEAST);
    double southwest = GetNeighbor(data, row, col, SOUTHWEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(value);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);
    accumulate(northeast);
    accumulate(northwest);
    accumulate(southeast);
    accumulate(southwest);

    mean /= nvals;

    if (value == valueToIgnore) value = mean;
    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;
    if (northeast == valueToIgnore) northeast = mean;
    if (northwest == valueToIgnore) northwest = mean;
    if (southeast == valueToIgnore) southeast = mean;
    if (southwest == valueToIgnore) southwest = mean;

    double zXX = (east - 2.0 * value + west) / (postSpacing * postSpacing);
    double zYY = (north - 2.0 * value + south) / (postSpacing * postSpacing);
    double zXY = ((-1.0 * northwest) + northeast + southwest - southeast) / (4.0 * postSpacing * postSpacing);
    double zX = (east - west) / (2 * postSpacing);
    double zY = (north - south) / (2 * postSpacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;

    return static_cast<float>(((zXX*zX*zX)-(2*zXY*zX*zY)+(zYY*zY*zY))/(p*std::sqrt(q*q*q)));
}


double DerivativeWriter::determineProfileCurvature(Eigen::MatrixXd* data,
        int row, int col, double postSpacing, double valueToIgnore)
{
    double mean = 0.0;
    unsigned int nvals = 0;

    double value = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);
    double northeast = GetNeighbor(data, row, col, NORTHEAST);
    double northwest = GetNeighbor(data, row, col, NORTHWEST);
    double southeast = GetNeighbor(data, row, col, SOUTHEAST);
    double southwest = GetNeighbor(data, row, col, SOUTHWEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(value);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);
    accumulate(northeast);
    accumulate(northwest);
    accumulate(southeast);
    accumulate(southwest);

    mean /= nvals;

    if (value == valueToIgnore) value = mean;
    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;
    if (northeast == valueToIgnore) northeast = mean;
    if (northwest == valueToIgnore) northwest = mean;
    if (southeast == valueToIgnore) southeast = mean;
    if (southwest == valueToIgnore) southwest = mean;

    double zXX = (east - 2.0 * value + west) / (postSpacing * postSpacing);
    double zYY = (north - 2.0 * value + south) / (postSpacing * postSpacing);
    double zXY = ((-1.0 * northwest) + northeast + southwest - southeast) / (4.0 * postSpacing * postSpacing);
    double zX = (east - west) / (2 * postSpacing);
    double zY = (north - south) / (2 * postSpacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;

    return static_cast<float>(((zXX*zX*zX)+(2*zXY*zX*zY)+(zYY*zY*zY))/(p*std::sqrt(q*q*q)));
}


double DerivativeWriter::determineTangentialCurvature(Eigen::MatrixXd* data,
        int row, int col, double postSpacing, double valueToIgnore)
{
    double mean = 0.0;
    unsigned int nvals = 0;

    double value = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);
    double northeast = GetNeighbor(data, row, col, NORTHEAST);
    double northwest = GetNeighbor(data, row, col, NORTHWEST);
    double southeast = GetNeighbor(data, row, col, SOUTHEAST);
    double southwest = GetNeighbor(data, row, col, SOUTHWEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(value);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);
    accumulate(northeast);
    accumulate(northwest);
    accumulate(southeast);
    accumulate(southwest);

    mean /= nvals;

    if (value == valueToIgnore) value = mean;
    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;
    if (northeast == valueToIgnore) northeast = mean;
    if (northwest == valueToIgnore) northwest = mean;
    if (southeast == valueToIgnore) southeast = mean;
    if (southwest == valueToIgnore) southwest = mean;

    double zXX = (east - 2.0 * value + west) / (postSpacing * postSpacing);
    double zYY = (north - 2.0 * value + south) / (postSpacing * postSpacing);
    double zXY = ((-1.0 * northwest) + northeast + southwest - southeast) / (4.0 * postSpacing * postSpacing);
    double zX = (east - west) / (2 * postSpacing);
    double zY = (north - south) / (2 * postSpacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;

    return static_cast<float>(((zXX*zY*zY)-(2*zXY*zX*zY)+(zYY*zX*zX))/(p*std::sqrt(q)));
}


double DerivativeWriter::determineTotalCurvature(Eigen::MatrixXd* data, int row,
        int col, double postSpacing, double valueToIgnore)
{
    double mean = 0.0;
    unsigned int nvals = 0;

    double value = static_cast<double>((*data)(row, col));
    double north = GetNeighbor(data, row, col, NORTH);
    double south = GetNeighbor(data, row, col, SOUTH);
    double east = GetNeighbor(data, row, col, EAST);
    double west = GetNeighbor(data, row, col, WEST);
    double northeast = GetNeighbor(data, row, col, NORTHEAST);
    double northwest = GetNeighbor(data, row, col, NORTHWEST);
    double southeast = GetNeighbor(data, row, col, SOUTHEAST);
    double southwest = GetNeighbor(data, row, col, SOUTHWEST);

    auto accumulate = [&nvals, &mean, valueToIgnore](double val)
    {
        if (val != valueToIgnore)
        {
            mean += val;
            nvals++;
        }
    };

    accumulate(value);
    accumulate(north);
    accumulate(south);
    accumulate(east);
    accumulate(west);
    accumulate(northeast);
    accumulate(northwest);
    accumulate(southeast);
    accumulate(southwest);

    mean /= nvals;

    if (value == valueToIgnore) value = mean;
    if (north == valueToIgnore) north = mean;
    if (south == valueToIgnore) south = mean;
    if (east == valueToIgnore) east = mean;
    if (west == valueToIgnore) west = mean;
    if (northeast == valueToIgnore) northeast = mean;
    if (northwest == valueToIgnore) northwest = mean;
    if (southeast == valueToIgnore) southeast = mean;
    if (southwest == valueToIgnore) southwest = mean;

    double zXX = (east - 2.0 * value + west) / (postSpacing * postSpacing);
    double zYY = (north - 2.0 * value + south) / (postSpacing * postSpacing);
    double zXY = ((-1.0 * northwest) + northeast + southwest - southeast) / (4.0 * postSpacing * postSpacing);

    return static_cast<float>((zXX * zXX) + (2.0 * zXY * zXY) + (zYY * zYY));
}


GDALDataset* DerivativeWriter::createFloat32GTIFF(std::string filename,
        int cols, int rows)
{
    char **papszMetadata;

    // parse the format driver, hardcoded for the time being
    std::string tFormat("GTIFF");
    const char *pszFormat = tFormat.c_str();
    GDALDriver* tpDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

    // try to create a file of the requested format
    if (tpDriver != NULL)
    {
        papszMetadata = tpDriver->GetMetadata();
        if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
        {
            char **papszOptions = NULL;

            pdalboost::filesystem::path p(filename);
            p.replace_extension(".tif");
            GDALDataset *dataset;
            dataset = tpDriver->Create(p.string().c_str(), cols, rows, 1,
                GDT_Float32, papszOptions);

            BOX2D& extent = getBounds();

            // set the geo transformation
            double adfGeoTransform[6];
            adfGeoTransform[0] = extent.minx; // - 0.5*m_GRID_DIST_X;
            adfGeoTransform[1] = m_GRID_DIST_X;
            adfGeoTransform[2] = 0.0;
            adfGeoTransform[3] = extent.maxy; // + 0.5*m_GRID_DIST_Y;
            adfGeoTransform[4] = 0.0;
            adfGeoTransform[5] = -1 * m_GRID_DIST_Y;
            dataset->SetGeoTransform(adfGeoTransform);

            // set the projection
            log()->get(LogLevel::Debug5) << m_inSRS.getWKT() << std::endl;
            dataset->SetProjection(m_inSRS.getWKT().c_str());

            if (dataset)
                return dataset;
        }
    }
    return NULL;
}


void DerivativeWriter::writeSlope(Eigen::MatrixXd* tDemData,
        const PointViewPtr data, PrimitiveType method,
        const std::string& filename)
{
    BOX2D& extent = getBounds();

    // use the max grid size as the post spacing
    double tPostSpacing = std::max(m_GRID_DIST_X, m_GRID_DIST_Y);

    GDALDataset *mpDstDS;
    mpDstDS = createFloat32GTIFF(filename, m_GRID_SIZE_X, m_GRID_SIZE_Y);

    // if we have a valid file
    if (mpDstDS)
    {
        // loop over the raster and determine max slope at each location
        int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
        int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
        float *poRasterData = new float[m_GRID_SIZE_X*m_GRID_SIZE_Y];
        for (uint32_t i=0; i<m_GRID_SIZE_X*m_GRID_SIZE_Y; i++)
        {
            poRasterData[i] = c_background;
        }

        #pragma omp parallel for
        for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
        {
            int tXIn = tXOut;
            for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
            {
                int tYIn = tYOut;

                float tSlopeValDegree(0);

                //Compute Slope Value
                switch (method)
                {
                    case SLOPE_D8:
                        tSlopeValDegree = (float)determineSlopeD8(tDemData,
                                          tYOut, tXOut, tPostSpacing,
                                          c_background);
                        break;

                    case SLOPE_FD:
                        tSlopeValDegree = (double)determineSlopeFD(tDemData,
                                          tYOut, tXOut, tPostSpacing,
                                          c_background);
                        break;
                    default:
                        assert(false);
                        return;
                }

                poRasterData[(tYIn * m_GRID_SIZE_X) + tXIn] =
                    std::tan(tSlopeValDegree*c_pi/180.0)*100.0;
            }
        }

        // write the data
        if (poRasterData)
        {
            GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

            tBand->SetNoDataValue((double)c_background);

            if (m_GRID_SIZE_X > 0 && m_GRID_SIZE_Y > 0)
// #define STR_HELPER(x) #x
// #define STR(x) STR_HELPER(x)
//
// #pragma message "content of GDAL_VERSION_MAJOR:" STR(GDAL_VERSION_MAJOR)
#if GDAL_VERSION_MAJOR <= 1
                tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                GDT_Float32, 0, 0);
#else

                int ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          GDT_Float32, 0, 0, 0);
#endif
        }

        GDALClose((GDALDatasetH) mpDstDS);

        delete [] poRasterData;
    }
}


void DerivativeWriter::writeAspect(Eigen::MatrixXd* tDemData,
    const PointViewPtr data, PrimitiveType method, const std::string& filename)
{
    BOX2D& extent = getBounds();

    // use the max grid size as the post spacing
    double tPostSpacing = std::max(m_GRID_DIST_X, m_GRID_DIST_Y);

    GDALDataset *mpDstDS;
    mpDstDS = createFloat32GTIFF(filename, m_GRID_SIZE_X, m_GRID_SIZE_Y);

    // if we have a valid file
    if (mpDstDS)
    {
        // loop over the raster and determine max slope at each location
        int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
        int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
        float *poRasterData = new float[m_GRID_SIZE_X*m_GRID_SIZE_Y];
        for (uint32_t i=0; i<m_GRID_SIZE_X*m_GRID_SIZE_Y; i++)
        {
            poRasterData[i] = 0;    // Initialize all elements to zero.
        }

        #pragma omp parallel for
        for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
        {
            int tXIn = tXOut;
            for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
            {
                int tYIn = tYOut;

                float tSlopeValDegree(0);

                //Compute Aspect Value
                switch (method)
                {
                    case ASPECT_D8:
                        tSlopeValDegree = (float)determineAspectD8(tDemData,
                                          tYOut, tXOut, tPostSpacing);
                        break;
                    case ASPECT_FD:
                        tSlopeValDegree = (float)determineAspectFD(tDemData,
                                tYOut, tXOut, tPostSpacing, c_background);
                        break;
                    default:
                        assert(false);
                        return;
                }

                if (tSlopeValDegree == std::numeric_limits<double>::max())
                    poRasterData[(tYIn * m_GRID_SIZE_X) + tXIn] = c_background;
                else
                    poRasterData[(tYIn * m_GRID_SIZE_X) + tXIn] =
                        tSlopeValDegree;
            }
        }

        // write the data
        if (poRasterData)
        {
            GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

            tBand->SetNoDataValue((double)c_background);

            if (m_GRID_SIZE_X > 0 && m_GRID_SIZE_Y > 0)
#if GDAL_VERSION_MAJOR <= 1
                tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                GDT_Float32, 0, 0);
#else
                int ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          GDT_Float32, 0, 0, 0);
#endif
        }

        GDALClose((GDALDatasetH) mpDstDS);

        delete [] poRasterData;
    }
}


void DerivativeWriter::writeCatchmentArea(Eigen::MatrixXd* tDemData,
        const PointViewPtr data, const std::string& filename)
{
    Eigen::MatrixXd area(m_GRID_SIZE_Y, m_GRID_SIZE_X);
    area.setZero();

    BOX2D& extent = getBounds();

    // use the max grid size as the post spacing
    double tPostSpacing = std::max(m_GRID_DIST_X, m_GRID_DIST_Y);

    GDALDataset *mpDstDS;
    mpDstDS = createFloat32GTIFF(filename, m_GRID_SIZE_X, m_GRID_SIZE_Y);

    // if we have a valid file
    if (mpDstDS)
    {
        // loop over the raster and determine max slope at each location
        int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
        int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
        float *poRasterData = new float[m_GRID_SIZE_X*m_GRID_SIZE_Y];
        for (uint32_t i=0; i<m_GRID_SIZE_X*m_GRID_SIZE_Y; i++)
        {
            poRasterData[i] = c_background;    // Initialize all elements to zero.
        }


        int tXOut = tXStart;
        int tYOut = tYStart;
        //for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
        //{
        //    for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
        //    {
        //Compute Aspect Value
        //switch (method)
        //{
        //case AD8:
        //tSlopeValDegree = (float)determineAspectD8(tDemData, tYOut, tXOut, tPostSpacing);
        //break;
        //
        //case SFD:
        //  tSlopeValDegree = (float)determineAspectFD(tDemData, tYOut, tXOut, tPostSpacing, c_background);
        //break;
        //}
        area(tYOut, tXOut) = determineCatchmentAreaD8(tDemData, &area, tYOut,
                             tXOut, tPostSpacing);
        //    }
        // }

        #pragma omp parallel for
        for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
        {
            for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
            {
                poRasterData[(tYOut * m_GRID_SIZE_X) + tXOut] = area(tYOut, tXOut);
            }
        }

        //stretchData(poRasterData);

        // write the data
        if (poRasterData)
        {
            GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

            tBand->SetNoDataValue((double)c_background);

            if (m_GRID_SIZE_X > 0 && m_GRID_SIZE_Y > 0)
#if GDAL_VERSION_MAJOR <= 1
                tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                GDT_Float32, 0, 0);
#else
                int ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          GDT_Float32, 0, 0, 0);
#endif
        }

        GDALClose((GDALDatasetH) mpDstDS);

        delete [] poRasterData;
    }
}

// void DerivativeWriter::stretchData(float *data)
// {
//     unsigned int nvals = 0;
//
//     int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
//     int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
//
//     // pass #1: compute mean
//     double mean = 0.0;
//     for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
//     {
//         for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
//         {
//             float val = data[(tYOut * m_GRID_SIZE_X) + tXOut];
//             if (val != c_background && !_isnanf(val))
//             {
//                 //std::cerr << val << std::endl;
//                 mean += val;
//                 nvals++;
//             }
//         }
//     }
//     mean /= nvals;
//
//     // pass #2: compute standard deviation
//     double stdev = 0.0;
//     for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
//     {
//         for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
//         {
//             float val = data[(tYOut * m_GRID_SIZE_X) + tXOut];
//             if (val != c_background && !_isnanf(val))
//             {
//                 stdev += std::pow(val - mean, 2);
//                 nvals++;
//             }
//         }
//     }
//     stdev /= (nvals - 1);
//     stdev = std::sqrt(stdev);
//
//     std::cerr << mean << ", " << stdev << ", " << nvals << std::endl;
//
//     // pass #3: scale to +/- 2x standard deviations from mean
//     double min_val = mean - 2*stdev;
//     double max_val = mean + 2*stdev;
//     double range = max_val - min_val;
//     double scale = 256.0 / range;
//
//     std::cerr << min_val << " < " << max_val << std::endl;
//
//     #pragma omp parallel for
//
//     for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
//     {
//         for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
//         {
//             data[(tYOut * m_GRID_SIZE_X) + tXOut] =
//                 (data[(tYOut * m_GRID_SIZE_X) + tXOut] - min_val) * scale;
//         }
//     }
// }


void DerivativeWriter::writeHillshade(Eigen::MatrixXd* tDemData,
    const PointViewPtr data, const std::string& filename)
{
    BOX2D& extent = getBounds();

    // use the max grid size as the post spacing
    double tPostSpacing = std::max(m_GRID_DIST_X, m_GRID_DIST_Y);

    GDALDataset *mpDstDS;
    mpDstDS = createFloat32GTIFF(filename, m_GRID_SIZE_X, m_GRID_SIZE_Y);

    // if we have a valid file
    if (mpDstDS)
    {
        // loop over the raster and determine max slope at each location
        int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
        int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
        float *poRasterData = new float[m_GRID_SIZE_X*m_GRID_SIZE_Y];
        for (uint32_t i=0; i<m_GRID_SIZE_X*m_GRID_SIZE_Y; i++)
        {
            poRasterData[i] = 0;    // Initialize all elements to zero.
        }

        // Parameters for hill shade
        double illumAltitudeDegree = 45.0;
        double illumAzimuthDegree = 315.0;
        double tZenithRad = (90 - illumAltitudeDegree) * (c_pi / 180.0);
        double tAzimuthMath = 360.0 - illumAzimuthDegree + 90;

        if (tAzimuthMath >= 360.0)
        {
            tAzimuthMath = tAzimuthMath - 360.0;
        }

        double tAzimuthRad = tAzimuthMath * (c_pi / 180.0);

        double min_val = std::numeric_limits<double>::max();
        double max_val = -std::numeric_limits<double>::max();

        #pragma omp parallel for
        for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
        {
            for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
            {
                //Compute Slope Value
                float tSlopeValDegree = (float)determineHillshade(tDemData,
                                        tYOut, tXOut, tZenithRad, tAzimuthRad,
                                        tPostSpacing);

                if (tSlopeValDegree == std::numeric_limits<double>::max())
                    poRasterData[(tYOut * m_GRID_SIZE_X) + tXOut] = c_background;
                else
                    poRasterData[(tYOut * m_GRID_SIZE_X) + tXOut] = tSlopeValDegree;

                if (tSlopeValDegree < min_val) min_val = tSlopeValDegree;
                if (tSlopeValDegree > max_val) max_val = tSlopeValDegree;
            }
        }

        // stretchData(poRasterData);

        // write the data
        if (poRasterData)
        {
            GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

            tBand->SetNoDataValue((double)c_background);

            if (m_GRID_SIZE_X > 0 && m_GRID_SIZE_Y > 0)
#if GDAL_VERSION_MAJOR <= 1
                tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                GDT_Float32, 0, 0);
#else

                int ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          poRasterData, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                                          GDT_Float32, 0, 0, 0);
#endif
        }

        GDALClose((GDALDatasetH) mpDstDS);

        delete [] poRasterData;
    }
}


void DerivativeWriter::writeCurvature(Eigen::MatrixXd* tDemData,
    const PointViewPtr data, PrimitiveType curveType, double valueToIgnore,
    const std::string& filename)
{
    BOX2D& extent = getBounds();

    // use the max grid size as the post spacing
    double tPostSpacing = std::max(m_GRID_DIST_X, m_GRID_DIST_Y);

    GDALDataset *mpDstDS;
    mpDstDS = createFloat32GTIFF(filename, m_GRID_SIZE_X, m_GRID_SIZE_Y);

    // if we have a valid file
    if (!mpDstDS)
        return;

    // loop over the raster and determine max slope at each location
    int tXStart = 1, tXEnd = m_GRID_SIZE_X - 1;
    int tYStart = 1, tYEnd = m_GRID_SIZE_Y - 1;
    std::vector<float> poRasterData(m_GRID_SIZE_X * m_GRID_SIZE_Y);
    for (uint32_t i=0; i<m_GRID_SIZE_X*m_GRID_SIZE_Y; i++)
    {
        poRasterData[i] = c_background;
    }

#pragma omp parallel for
    for (int tXOut = tXStart; tXOut < tXEnd; tXOut++)
    {
        int tXIn = tXOut;
        for (int tYOut = tYStart; tYOut < tYEnd; tYOut++)
        {
            int tYIn = tYOut;

            double curve(0);

            //Compute Slope Value
            switch (curveType)
            {
                case CONTOUR_CURVATURE:
                    curve = determineContourCurvature(tDemData,
                            tYOut, tXOut,
                            tPostSpacing,
                            c_background);
                    break;

                case PROFILE_CURVATURE:
                    curve = determineProfileCurvature(tDemData,
                            tYOut, tXOut,
                            tPostSpacing,
                            c_background);
                    break;

                case TANGENTIAL_CURVATURE:
                    curve = determineTangentialCurvature(tDemData,
                            tYOut, tXOut,
                            tPostSpacing,
                            c_background);
                    break;

                case TOTAL_CURVATURE:
                    curve = determineTotalCurvature(tDemData,
                            tYOut, tXOut,
                            tPostSpacing,
                            c_background);
                    break;
                default:
                    assert(false);
                    return;
            }

            poRasterData[(tYOut * m_GRID_SIZE_X) + tXOut] =
                static_cast<float>(curve);
        }
    }

    //stretchData(poRasterData);

    // write the data
    GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

    tBand->SetNoDataValue((double)c_background);

    if (m_GRID_SIZE_X > 0 && m_GRID_SIZE_Y > 0)
    {
        int ret;
#if GDAL_VERSION_MAJOR <= 1
        ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
                poRasterData.data(), m_GRID_SIZE_X, m_GRID_SIZE_Y,
                GDT_Float32, 0, 0);
#else
        ret = tBand->RasterIO(GF_Write, 0, 0, m_GRID_SIZE_X, m_GRID_SIZE_Y,
            poRasterData.data(), m_GRID_SIZE_X, m_GRID_SIZE_Y,
            GDT_Float32, 0, 0, 0);
#endif
        if (ret != CE_None)
        {
            std::ostringstream oss;

            oss << getName() << ": Error reading raster IO.";
            throw pdal_error(oss.str());
        }
    }
    GDALClose((GDALDatasetH) mpDstDS);
}


void DerivativeWriter::write(const PointViewPtr data)
{
    m_inSRS = data->spatialReference();
    data->calculateBounds(m_bounds);

    // calculate grid based off bounds and post spacing
    calculateGridSizes();
    log()->get(LogLevel::Debug2) << "X grid size: " <<
        m_GRID_SIZE_X << std::endl;
    log()->get(LogLevel::Debug2) << "Y grid size: " <<
        m_GRID_SIZE_Y << std::endl;

    log()->floatPrecision(6);
    log()->get(LogLevel::Debug2) << "X grid distance: " <<
        m_GRID_DIST_X << std::endl;
    log()->get(LogLevel::Debug2) << "Y grid distance: " <<
        m_GRID_DIST_Y << std::endl;
    log()->clearFloat();

    BOX2D& extent = getBounds();
    double yMax = extent.miny + m_GRID_SIZE_Y * m_GRID_DIST_Y;
    log()->get(LogLevel::Debug4) << yMax << ", " << extent.maxy << std::endl;

    // need to create the min DEM
    Eigen::MatrixXd tDemData(m_GRID_SIZE_Y, m_GRID_SIZE_X);
    tDemData.setConstant(c_background);
    for (PointId idx = 0; idx < data->size(); ++idx)
    {
        double x = data->getFieldAs<double>(Dimension::Id::X, idx);
        double y = data->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = data->getFieldAs<double>(Dimension::Id::Z, idx);

        auto clamp = [](double t, double min, double max)
        {
            return ((t < min) ? min : ((t > max) ? max : t));
        };

        int xIndex = clamp(static_cast<int>(floor((x - extent.minx) / m_GRID_DIST_X)), 0, m_GRID_SIZE_X-1);
        int yIndex = clamp(static_cast<int>(floor((yMax - y) / m_GRID_DIST_Y)), 0, m_GRID_SIZE_Y-1);

        double tDemValue = tDemData(yIndex, xIndex);

        if (tDemValue == c_background)
        {
            tDemData(yIndex, xIndex) = z;
        }
        else
        {
            if (z > tDemValue)
                tDemData(yIndex, xIndex) = z;
        }
    }

    auto CleanRasterScanLine = [](Eigen::MatrixXd data, Eigen::VectorXd datarow,
                                  int mDim, int row, bool* prevSetCols,
                                  bool* curSetCols)
    {

        auto InterpolateRasterPixelScanLine = [](Eigen::MatrixXd data, int mDim,
                                              int x, int y, bool* prevSetCols)
        {
            int yMinus, yPlus, xMinus, xPlus;
            float tInterpValue;
            bool tPrevInterp;

            yMinus = y - 1;
            yPlus = y + 1;
            xMinus = x - 1;
            xPlus = x + 1;

            //North
            tInterpValue = data(yMinus, x);
            tPrevInterp = prevSetCols[x];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //South
            tInterpValue = data(yPlus, x);
            if (tInterpValue != c_background)
                return tInterpValue;

            //East
            tInterpValue = data(y, xPlus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //West
            tInterpValue = data(y, xMinus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //NorthWest
            tInterpValue = data(yMinus, xMinus);
            tPrevInterp = prevSetCols[xMinus];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //NorthWest
            tInterpValue = data(yMinus, xPlus);
            tPrevInterp = prevSetCols[xPlus];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //SouthWest
            tInterpValue = data(yPlus, xMinus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //SouthEast
            tInterpValue = data(yPlus, xPlus);
            if (tInterpValue != c_background)
                return tInterpValue;

            return 0.0f;
        };

        float tInterpValue;
        float tValue;

        int y = row;
        for (int x = 1; x < mDim-1; ++x)
        {
            tValue = datarow(x);

            if (tValue == c_background)
            {
                tInterpValue = InterpolateRasterPixelScanLine(data, mDim, x, y,
                               prevSetCols);
                if (tInterpValue != c_background)
                {
                    curSetCols[x] = true;
                    datarow(x) = tInterpValue;
                }
            }
        }
    };

    {
        std::unique_ptr<bool> prevSetCols(new bool[m_GRID_SIZE_X]);
        std::unique_ptr<bool> curSetCols(new bool[m_GRID_SIZE_X]);

        for (uint32_t y = 1; y < m_GRID_SIZE_Y-1; ++y)
        {
            CleanRasterScanLine(tDemData, tDemData.row(1), m_GRID_SIZE_X, y,
                prevSetCols.get(), curSetCols.get());
            memcpy(prevSetCols.get(), curSetCols.get(), m_GRID_SIZE_X);
            memset(curSetCols.get(), 0, m_GRID_SIZE_X);
        }
    }

    for (TypeOutput& to : m_primitiveTypes)
    {
        switch (to.m_type)
        {
        case SLOPE_D8:
        case SLOPE_FD:
            writeSlope(&tDemData, data, to.m_type, to.m_filename);
            break;
        case ASPECT_D8:
        case ASPECT_FD:
            writeAspect(&tDemData, data, to.m_type, to.m_filename);
            break;
        case HILLSHADE:
            writeHillshade(&tDemData, data, to.m_filename);
            break;
        case CONTOUR_CURVATURE:
        case PROFILE_CURVATURE:
        case TANGENTIAL_CURVATURE:
        case TOTAL_CURVATURE:
            writeCurvature(&tDemData, data, to.m_type, c_background,
                to.m_filename);
            break;
        case CATCHMENT_AREA:
            writeCatchmentArea(&tDemData, data, to.m_filename);
            break;
        }
    }
}

void DerivativeWriter::calculateGridSizes()
{
    BOX2D& extent = getBounds();

    m_GRID_SIZE_X = (int)(ceil((extent.maxx - extent.minx)/m_GRID_DIST_X)) + 1;
    m_GRID_SIZE_Y = (int)(ceil((extent.maxy - extent.miny)/m_GRID_DIST_Y)) + 1;
}

} // namespace pdal
