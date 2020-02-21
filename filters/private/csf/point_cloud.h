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

// #######################################################################################
// #                                                                                     #
// #            CSF: Airborne LiDAR filtering based on Cloth Simulation                  #
// #                                                                                     #
// #  Please cite the following paper, If you use this software in your work.            #
// #                                                                                     #
// #  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
// #  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
// #                               (http://ramm.bnu.edu.cn/)                             #
// #                                                                                     #
// #                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
// #                                                                                     #
// #                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
// #                                                                                     #
// #######################################################################################

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <vector>

namespace csf {

struct Point {
    union {
        struct {
            double x;
			double y;
			double z;
        };
		double u[3];
    };

    Point() : x(0), y(0), z(0) {}
};

class PointCloud : public std::vector<Point>{
public:

    void computeBoundingBox(Point& bbMin, Point& bbMax);
};

}


#endif // ifndef _POINT_CLOUD_H_
