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

#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_


#include "Vec3.h"
#include "Particle.h"

const double singleMove[14] = { 0.4, 0.64, 0.784, 0.8704, 0.92224, 0.95334, 0.97201, 0.9832, 0.98992, 0.99395, 0.99637, 0.99782, 0.99869, 0.99922 };

const double doubleMove[14] = { 0.4, 0.48, 0.496, 0.4992, 0.49984, 0.49997, 0.49999, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

class Constraint {
private:

    double rest_distance; // the length between particle p1 and p2 in rest configuration

public:

    Particle *p1, *p2; // the two particles that are connected through this constraint

    Constraint(Particle *p1, Particle *p2) : p1(p1), p2(p2) {}

    /* This is one of the important methods, where a single constraint
     * between two particles p1 and p2 is solved the method is called
     * by Cloth.time_step() many times per frame*/
    void satisfyConstraint(int constraintTimes);
};


#endif // ifndef _CONSTRAINT_H_
