/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#pragma once

#include <limits>

namespace pdal { namespace greyhound {

class Range
{
public:
    Range()
        : min(std::numeric_limits<double>::max())
        , max(std::numeric_limits<double>::lowest())
    { }

    Range(double min, double max) : min(min), max(max) { }

    void grow(double val)
    {
        min = std::min(min, val);
        max = std::max(max, val);
    }

    double min;
    double max;
};

} // namespace entwine
} // namepsace pdal
