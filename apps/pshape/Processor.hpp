#pragma once

#include <boost/function.hpp>
#include <vector>

#include "GridInfo.hpp"
#include "Path.hpp"

namespace Pshape
{
    typedef boost::function<bool(double&, double&)> PointReader;
    typedef boost::function<bool(int&, int&)> HexReader;

    void process(const std::vector<GridInfo *>& infos, PointReader);
    void processHexes(const std::vector<GridInfo *>& infos, HexReader);
} // namespace Pshape
