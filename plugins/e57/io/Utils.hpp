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

#pragma once

#include <string>
#include <E57Format.h>
#include <pdal/Dimension.hpp>

namespace pdal
{
namespace e57plugin
{
    // converts an e57 dimension string to a pdal dimension
    // returns pdal::Dimension::Id::Unknown in case the dimension is not recognised
    pdal::Dimension::Id e57ToPdal(const std::string &e57Dimension);

    // converts a pdal dimension to the corresponding E57 string
    // returns an empty string in case the dimension is not recognised
    std::string pdalToE57(pdal::Dimension::Id pdalDimension);

    /// Converts a value from E57 to pdal. Handles change in type representation
    /// For example, intensity in e57 is between 0 and 1 and 0 and 2^16 in pdal
    double
    rescaleE57ToPdalValue(const std::string &e57Dimension, double value, const std::pair<double, double> &e57Bounds);

    std::vector<pdal::Dimension::Id> supportedPdalTypes();
    std::vector<std::string> supportedE57Types();

    // Tries to find the limit of a dimension in the e57 node headers
    // return nan if not found
    std::pair<double, double> getLimits(const e57::StructureNode &prototype, const std::string &fieldName);

    // Get the bounds of a given dimension as expected by pdal
    std::pair<double,double> getPdalBounds(pdal::Dimension::Id id);
}
}
