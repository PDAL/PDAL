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

#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>

using namespace std;

void read_xyz(string fname, csf::PointCloud& pointcloud) {
    ifstream fin(fname.c_str(), ios::in);
    char     line[500];
    string   x, y, z;

    while (fin.getline(line, sizeof(line))) {
        stringstream words(line);

        words >> x;
        words >> y;
        words >> z;

        csf::Point point;
        point.x = atof(x.c_str());
        point.y = -atof(z.c_str());
        point.z = atof(y.c_str());

        pointcloud.push_back(point);
    }
}
