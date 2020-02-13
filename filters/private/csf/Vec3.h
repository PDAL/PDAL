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

#ifndef _VEC3_H_
#define _VEC3_H_


#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

using namespace std;

// a minimal vector class of 3 doubles and overloaded math operators
class Vec3 {
public:

    double f[3];

    Vec3(double x, double y, double z) {
        f[0] = x;
        f[1] = y;
        f[2] = z;
    }

    Vec3() {}

    double length() {
        return sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    }

    Vec3 normalized() {
        double l = length();

        return Vec3(f[0] / l, f[1] / l, f[2] / l);
    }

    void operator+=(const Vec3& v) {
        f[0] += v.f[0];
        f[1] += v.f[1];
        f[2] += v.f[2];
    }

    Vec3 operator/(const double& a) {
        return Vec3(f[0] / a, f[1] / a, f[2] / a);
    }

    Vec3 operator-(const Vec3& v) {
        return Vec3(f[0] - v.f[0], f[1] - v.f[1], f[2] - v.f[2]);
    }

    Vec3 operator+(const Vec3& v) {
        return Vec3(f[0] + v.f[0], f[1] + v.f[1], f[2] + v.f[2]);
    }

    Vec3 operator*(const double& a) {
        return Vec3(f[0] * a, f[1] * a, f[2] * a);
    }

    Vec3 operator-() {
        return Vec3(-f[0], -f[1], -f[2]);
    }

    Vec3 cross(const Vec3& v) {
        return Vec3(
            f[1] * v.f[2] - f[2] * v.f[1],
            f[2] * v.f[0] - f[0] * v.f[2],
            f[0] * v.f[1] - f[1] * v.f[0]
        );
    }

    double dot(const Vec3& v) {
        return f[0] * v.f[0] + f[1] * v.f[1] + f[2] * v.f[2];
    }
};


#endif // ifndef _VEC3_H_
