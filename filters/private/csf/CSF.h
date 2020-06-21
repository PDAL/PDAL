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


// cloth simulation filter for airborne lidar filtering
#ifndef _CSF_H_
#define _CSF_H_
#include <vector>
#include <string>
#include "point_cloud.h"
#include <pdal/Log.hpp>
using namespace std;


struct Params {
    // refer to the website:http://ramm.bnu.edu.cn/projects/CSF/ for the setting of these paramters
    bool bSloopSmooth;
    double time_step;
    double class_threshold;
    double cloth_resolution;
    int rigidness;
    int interations;
};

#ifdef _CSF_DLL_EXPORT_
# ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllexport)
# else // ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllimport)
# endif // ifdef DLL_IMPLEMENT
#endif // ifdef _CSF_DLL_EXPORT_

#ifdef _CSF_DLL_EXPORT_
class DLL_API CSF
#else // ifdef _CSF_DLL_EXPORT_
class CSF
#endif // ifdef _CSF_DLL_EXPORT_
{
public:

    CSF(int index);
    CSF();
    ~CSF();

    // set pointcloud from vector
    void setPointCloud(vector<csf::Point> points);
    // set point cloud from a one-dimentional array. it defines a N*3 point cloud by the given rows.
    void setPointCloud(double *points, int rows);

    // set point cloud for python
    void setPointCloud(vector<vector<float> > points);

    // read pointcloud from txt file: (X Y Z) for each line
    void readPointsFromFile(string filename);

    inline csf::PointCloud& getPointCloud() {
        return point_cloud;
    }

    inline const csf::PointCloud& getPointCloud() const {
        return point_cloud;
    }

    // save points to file
    void savePoints(vector<int> grp, string path);

    // get size of pointcloud
    size_t size() {
        return point_cloud.size();
    }

    // PointCloud set pointcloud
    void setPointCloud(csf::PointCloud& pc);

    // set Log
    void setLog(const pdal::LogPtr& log);

    // The results are index of ground points in the original
    // pointcloud
    void do_filtering(std::vector<int>& groundIndexes,
                      std::vector<int>& offGroundIndexes,
                      bool              exportCloth = false);

private:

#ifdef _CSF_DLL_EXPORT_
    class __declspec (dllexport)csf::PointCloud point_cloud;
#else // ifdef _CSF_DLL_EXPORT_
    csf::PointCloud point_cloud;
#endif // ifdef _CSF_DLL_EXPORT_
    pdal::LogPtr log;

public:

    Params params;
    int index;
};

#endif // ifndef _CSF_H_
