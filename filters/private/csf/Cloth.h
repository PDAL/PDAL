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

/*
 * This source code is about a ground filtering algorithm for airborn LiDAR data
 * based on physical process simulations, specifically cloth simulation.
 *
 * this code is based on a Cloth Simulation Tutorial at the cg.alexandra.dk blog.
 * Thanks to Jesper Mosegaard (clothTutorial@jespermosegaard.dk)
 *
 *
 *
 * When applying the cloth simulation to LIDAR point filtering. A lot of features
 * have been added to the original source code, including
 * configuration file management
 * point cloud data read/write
 * point-to-point collsion detection
 * nearest point search structure from CGAL
 * addding a terrain class
 *
 *
 */
// using discrete steps (drop and pull) to approximate the physical process

#ifndef _CLOTH_H_
#define _CLOTH_H_


#ifdef _WIN32
# include <windows.h>
#endif // ifdef _WIN32
#include <math.h>
#include <vector>
#include <iostream>
//#include <omp.h>
#include <iostream>
#include <sstream>
#include <list>
#include <cmath>
#include <vector>
#include <string>
#include <list>
#include <queue>
#include <cmath>
#include <list>
using namespace std;

#include "Vec3.h"
#include "Particle.h"
// #include <boost/progress.hpp>
// post processing is only for connected component which is large than 50
#define MAX_PARTICLE_FOR_POSTPROCESSIN    50

struct XY {
    XY(int x1, int y1) {
        x = x1; y = y1;
    }

    int x;
    int y;
};

class Cloth {
private:

    // total number of particles is num_particles_width * num_particles_height
    int constraint_iterations;

     // all particles that are part of this cloth
    std::vector<Particle> particles;

    double smoothThreshold;
    double heightThreshold;

public:

    Vec3 origin_pos;
    double step_x, step_y;
    vector<double> heightvals; // height values
    int num_particles_width;   // number of particles in width direction
    int num_particles_height;  // number of particles in height direction

    Particle* getParticle(int x, int y) {
        return &particles[y * num_particles_width + x];
    }

    void makeConstraint(Particle *p1, Particle *p2) {
        p1->neighborsList.push_back(p2);
        p2->neighborsList.push_back(p1);
    }

public:

    int getSize() {
        return num_particles_width * num_particles_height;
    }

    size_t get1DIndex(int x, int y) {
        return y * num_particles_width + x;
    }

    inline std::vector<double>& getHeightvals() {
        return heightvals;
    }

    Particle* getParticle1d(int index) {
        return &particles[index];
    }

public:

    /* This is a important constructor for the entire system of
     * particles and constraints */
    Cloth(const Vec3& origin_pos,
          int         num_particles_width,
          int         num_particles_height,
          double      step_x,
          double      step_y,
          double      smoothThreshold,
          double      heightThreshold,
          int         rigidness,
          double      time_step);

    /* this is an important methods where the time is progressed one
     * time step for the entire cloth.  This includes calling
     * satisfyConstraint() for every constraint, and calling
     * timeStep() for all particles
     */
    double timeStep();

    /* used to add gravity (or any other arbitrary vector) to all
     * particles */
    void addForce(const Vec3 direction);

    void terrCollision();

    void movableFilter();

    vector<int> findUnmovablePoint(vector<XY> connected);

    void        handle_slop_connected(vector<int>          edgePoints,
                                      vector<XY>           connected,
                                      vector<vector<int> > neibors);

    void saveToFile(string path = "");

    void saveMovableToFile(string path = "");
};


#endif // ifndef _CLOTH_H_
