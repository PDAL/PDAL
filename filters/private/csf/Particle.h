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

#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include "Vec3.h"
#include <vector>
/* Some physics constants */
#define DAMPING    0.01 // how much to damp the cloth simulation each frame
#define MAX_INF    9999999999
#define MIN_INF    -9999999999


const double singleMove1[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
const double doubleMove1[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

/* The particle class represents a particle of mass that can move
 * around in 3D space*/
class Particle {
private:

    bool movable;            // can the particle move or not ? used to pin parts of the cloth
    double mass;             // the mass of the particle (is always 1 in this example)
    Vec3 acceleration;       // a vector representing the current acceleration of the particle
    Vec3 accumulated_normal; // an accumulated normal (i.e. non normalized), used for OpenGL soft shading
    double time_step2;

public:

    // These two memebers are used in the process of edge smoothing after
    // the cloth simulation step.
    Vec3 pos;     // the current position of the particle in 3D space
    // the position of the particle in the previous time step, used as
    // part of the verlet numerical integration scheme
    Vec3 old_pos;
    bool isVisited;
    int neibor_count;
    int pos_x; // position in the cloth grid
    int pos_y;
    int c_pos;

    vector<Particle *> neighborsList;

    std::vector<int> correspondingLidarPointList;
    std::size_t nearestPointIndex;
    double nearestPointHeight;
    double tmpDist;
    void satisfyConstraintSelf(int constraintTimes);

public:

    Particle(Vec3 pos, double time_step) :
        movable(true),
        mass(1),
        acceleration(Vec3(0, 0, 0)),
        accumulated_normal(Vec3(0, 0, 0)),
        time_step2(time_step),
        pos(pos),
        old_pos(pos) {
        isVisited          = false;
        neibor_count       = 0;
        pos_x              = 0;
        pos_y              = 0;
        c_pos              = 0;
        nearestPointHeight = MIN_INF;
        tmpDist            = MAX_INF;
    }

    Particle() :
      movable(true),
      mass(1),
      acceleration(Vec3(0, 0, 0)),
      accumulated_normal(Vec3(0, 0, 0)),
      pos(Vec3(0, 0, 0)),
      old_pos(Vec3(0, 0, 0)) {
      isVisited          = false;
      neibor_count       = 0;
      pos_x              = 0;
      pos_y              = 0;
      c_pos              = 0;
      nearestPointHeight = MIN_INF;
      tmpDist            = MAX_INF;
    }


    bool isMovable() {
        return movable;
    }

    void addForce(Vec3 f) {
        acceleration += f / mass;
    }

    /* This is one of the important methods, where the time is
     * progressed a single step size (TIME_STEPSIZE) The method is
     * called by Cloth.time_step()*/
    void  timeStep();

    Vec3& getPos() {
        return pos;
    }

    Vec3 getPosCopy() {
        return pos;
    }

    void resetAcceleration() {
        acceleration = Vec3(0, 0, 0);
    }

    void offsetPos(const Vec3 v) {
        if (movable) pos += v;
    }

    void makeUnmovable() {
        movable = false;
    }

    void addToNormal(Vec3 normal) {
        accumulated_normal += normal.normalized();
    }

    Vec3& getNormal() {
        // note: The normal is not unit length
        return accumulated_normal;
    }

    void resetNormal() {
        accumulated_normal = Vec3(0, 0, 0);
    }

    // unused
    //void printself(string s = "") {
    //    cout << s << ": " << this->getPos().f[0] << " movable:  " << this->movable << endl;
    //}
};


#endif // ifndef _PARTICLE_H_
