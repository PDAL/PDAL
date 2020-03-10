// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#ifndef _KNN_H_
#define _KNN_H_

#include "point_cloud.h"
#include "Cloth.h"

#define SQUARE_DIST(x1, y1, x2, y2) \
    (((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))


class Rasterization {
public:

    Rasterization() {}
    ~Rasterization() {}

    // for a cloth particle, if no corresponding lidar point are found.
    // the heightval are set as its neighbor's
    double static findHeightValByNeighbor(Particle *p, Cloth& cloth);
    double static findHeightValByScanline(Particle *p, Cloth& cloth);

    void static   RasterTerrian(Cloth          & cloth,
                                csf::PointCloud& pc,
                                vector<double> & heightVal);
};

#endif // ifndef _KNN_H_
