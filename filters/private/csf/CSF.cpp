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

#define DLL_IMPLEMENT

#include "CSF.h"
#include "XYZReader.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "c2cdist.h"
#include <fstream>
#include <pdal/Log.hpp>


CSF::CSF(int index) {
    params.bSloopSmooth     = true;
    params.time_step        = 0.65;
    params.class_threshold  = 0.5;
    params.cloth_resolution = 1;
    params.rigidness        = 3;
    params.interations      = 500;

    this->index = index;
}

CSF::CSF() {
    params.bSloopSmooth     = true;
    params.time_step        = 0.65;
    params.class_threshold  = 0.5;
    params.cloth_resolution = 1;
    params.rigidness        = 3;
    params.interations      = 500;

    this->index = 0;
}

CSF::~CSF()
{}

void CSF::setPointCloud(vector<csf::Point> points) {
    point_cloud.resize(points.size());

    int pointCount = static_cast<int>(points.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x          = points[i].x;
        las.y          = -points[i].z;
        las.z          = points[i].y;
        point_cloud[i] = las;
    }
}

void CSF::setPointCloud(double *points, int rows) {
    #define A(i, j) points[i + j * rows]

    for (int i = 0; i < rows; i++) {
        csf::Point p;
        p.x = A(i, 0);
        p.y = -A(i, 2);
        p.z = A(i, 1);
        point_cloud.push_back(p);
    }
}

void CSF::setLog(const pdal::LogPtr& l) {
    log = l;
}

void CSF::setPointCloud(csf::PointCloud& pc) {
    log->get(pdal::LogLevel::Debug) << "setPointCloud: " << pc.size() << endl;
    point_cloud.resize(pc.size());
    int pointCount = static_cast<int>(pc.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x          = pc[i].x;
        las.y          = -pc[i].z;
        las.z          = pc[i].y;
        point_cloud[i] = las;
    }
}

void CSF::setPointCloud(vector<vector<float> > points) {
    point_cloud.resize(points.size());
    int pointCount = static_cast<int>(points.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x          = points[i][0];
        las.y          = -points[i][2];
        las.z          = points[i][1];
        point_cloud[i] = las;
    }
}

void CSF::readPointsFromFile(string filename) {
    this->point_cloud.resize(0);
    read_xyz(filename, this->point_cloud);
}


void CSF::do_filtering(std::vector<int>& groundIndexes,
                       std::vector<int>& offGroundIndexes,
                       bool              exportCloth) {
    // Terrain
    log->get(pdal::LogLevel::Debug) << "[" << this->index << "] Configuring terrain..." << endl;
    csf::Point bbMin, bbMax;
    point_cloud.computeBoundingBox(bbMin, bbMax);
    log->get(pdal::LogLevel::Debug) << bbMax.x << ", " << bbMin.x << endl;

    double cloth_y_height = 0.05;

    int  clothbuffer_d = 2;
    Vec3 origin_pos(
        bbMin.x - clothbuffer_d *params.cloth_resolution,
        bbMax.y + cloth_y_height,
        bbMin.z - clothbuffer_d *params.cloth_resolution
    );

    int width_num = static_cast<int>(
        floor((bbMax.x - bbMin.x) / params.cloth_resolution)
    ) + 2 * clothbuffer_d;

    int height_num = static_cast<int>(
        floor((bbMax.z - bbMin.z) / params.cloth_resolution)
    ) + 2 * clothbuffer_d;

    log->get(pdal::LogLevel::Debug) << "[" << this->index << "] Configuring cloth..." << endl;
    log->get(pdal::LogLevel::Debug) << "[" << this->index << "]  - width: " << width_num << " "
                                    << "height: " << height_num << endl;

    Cloth cloth(
        origin_pos,
        width_num,
        height_num,
        params.cloth_resolution,
        params.cloth_resolution,
        0.3,
        9999,
        params.rigidness,
        params.time_step
    );

    log->get(pdal::LogLevel::Debug) << "[" << this->index << "] Rasterizing..." << endl;
    Rasterization::RasterTerrian(cloth, point_cloud, cloth.getHeightvals());

    double time_step2 = params.time_step * params.time_step;
    double gravity    = 0.2;

    log->get(pdal::LogLevel::Debug) << "[" << this->index << "] Simulating..." << endl;
    cloth.addForce(Vec3(0, -gravity, 0) * time_step2);

    // boost::progress_display pd(params.interations);
    for (int i = 0; i < params.interations; i++) {
        double maxDiff = cloth.timeStep();
        cloth.terrCollision();
		//params.class_threshold / 100
        if ((maxDiff != 0) && (maxDiff < 0.005)) {
            // early stop
            break;
        }
        // pd++;
    }

    if (params.bSloopSmooth) {
        log->get(pdal::LogLevel::Debug) << "[" << this->index << "]  - post handle..." << endl;
        cloth.movableFilter();
    }

    if (exportCloth)
        cloth.saveToFile();

    c2cdist c2c(params.class_threshold);
    c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
}

void CSF::savePoints(vector<int> grp, string path) {
    if (path == "") {
        return;
    }

    ofstream f1(path.c_str(), ios::out);

    if (!f1)
        return;

    for (std::size_t i = 0; i < grp.size(); i++) {
        f1 << fixed << setprecision(8)
           << point_cloud[grp[i]].x  << "	"
           << point_cloud[grp[i]].z  << "	"
           << -point_cloud[grp[i]].y << endl;
    }

    f1.close();
}
