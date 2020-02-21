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

#ifndef _C2CDIST_H_
#define _C2CDIST_H_


#include "Cloth.h"
#include "point_cloud.h"


class c2cdist {
public:

    c2cdist(double threshold) : class_treshold(threshold) {}

    ~c2cdist() {}

public:

    void calCloud2CloudDist(Cloth           & cloth,
                            csf::PointCloud & pc,
                            std::vector<int>& groundIndexes,
                            std::vector<int>& offGroundIndexes);

private:

    double class_treshold; //
};


#endif // ifndef _C2CDIST_H_
