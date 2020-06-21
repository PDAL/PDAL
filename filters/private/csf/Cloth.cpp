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

#include "Cloth.h"
#include <fstream>


Cloth::Cloth(const Vec3& _origin_pos,
             int         _num_particles_width,
             int         _num_particles_height,
             double      _step_x,
             double      _step_y,
             double      _smoothThreshold,
             double      _heightThreshold,
             int         rigidness,
             double      time_step)
    : constraint_iterations(rigidness),
    smoothThreshold(_smoothThreshold),
    heightThreshold(_heightThreshold),
    origin_pos(_origin_pos),
    step_x(_step_x),
    step_y(_step_y),
    num_particles_width(_num_particles_width),
    num_particles_height(_num_particles_height) {
    // I am essentially using this vector as an array with room for
    // num_particles_width*num_particles_height particles
    particles.resize(num_particles_width * num_particles_height);

    double time_step2 = time_step * time_step;

    // creating particles in a grid of particles from (0,0,0) to
    // (width,-height,0) creating particles in a grid
    for (int i = 0; i < num_particles_width; i++) {
        for (int j = 0; j < num_particles_height; j++) {
            Vec3 pos(origin_pos.f[0] + i *step_x,
                     origin_pos.f[1],
                     origin_pos.f[2] + j *step_y);

            // insert particle in column i at j'th row
            particles[j * num_particles_width + i]       = Particle(pos, time_step2);
            particles[j * num_particles_width + i].pos_x = i;
            particles[j * num_particles_width + i].pos_y = j;
        }
    }

    saveToFile("initial-nodes.txt");

    // Connecting immediate neighbor particles with constraints
    // (distance 1 and sqrt(2) in the grid)
    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 1)
                makeConstraint(getParticle(x, y), getParticle(x + 1, y));

            if (y < num_particles_height - 1)
                makeConstraint(getParticle(x, y), getParticle(x, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                makeConstraint(getParticle(x, y), getParticle(x + 1, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                makeConstraint(getParticle(x + 1, y), getParticle(x, y + 1));
        }
    }

    // Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in the grid)
    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 2)
                makeConstraint(getParticle(x, y), getParticle(x + 2, y));

            if (y < num_particles_height - 2)
                makeConstraint(getParticle(x, y), getParticle(x, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x, y), getParticle(x + 2, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x + 2, y), getParticle(x, y + 2));
        }
    }
}

double Cloth::timeStep() {
    int particleCount = static_cast<int>(particles.size());
    #pragma omp parallel for
    for (int i = 0; i < particleCount; i++) {
        particles[i].timeStep();
    }
    #pragma omp parallel for
    for (int j = 0; j < particleCount; j++) {
        particles[j].satisfyConstraintSelf(constraint_iterations);
    }
    double maxDiff = 0;

    for (int i = 0; i < particleCount; i++) {
        if (particles[i].isMovable()) {
            double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);

            if (diff > maxDiff)
                maxDiff = diff;
        }
    }

    return maxDiff;
}

void Cloth::addForce(const Vec3 direction) {
    for (std::size_t i = 0; i < particles.size(); i++) {
        particles[i].addForce(direction);
    }
    saveToFile("force-nodes.txt");
}

void Cloth::terrCollision() {
    int particleCount = static_cast<int>(particles.size());
    #pragma omp parallel for
    for (int i = 0; i < particleCount; i++) {
        Vec3 v = particles[i].getPos();

        if (v.f[1] < heightvals[i]) {
            particles[i].offsetPos(Vec3(0, heightvals[i] - v.f[1], 0));
            particles[i].makeUnmovable();
        }
    }
    saveToFile("collision-notes.txt");
}

void Cloth::movableFilter() {
    vector<Particle> tmpParticles;

    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            Particle *ptc = getParticle(x, y);

            if (ptc->isMovable() && !ptc->isVisited) {
                queue<int> que;
                vector<XY> connected; // store the connected component
                vector<vector<int> > neibors;
                int sum   = 1;
                int index = y * num_particles_width + x;

                // visit the init node
                connected.push_back(XY(x, y));
                particles[index].isVisited = true;

                // enqueue the init node
                que.push(index);

                while (!que.empty()) {
                    Particle *ptc_f = &particles[que.front()];
                    que.pop();
                    int cur_x = ptc_f->pos_x;
                    int cur_y = ptc_f->pos_y;
                    vector<int> neibor;

                    if (cur_x > 0) {
                        Particle *ptc_left = getParticle(cur_x - 1, cur_y);

                        if (ptc_left->isMovable()) {
                            if (!ptc_left->isVisited) {
                                sum++;
                                ptc_left->isVisited = true;
                                connected.push_back(XY(cur_x - 1, cur_y));
                                que.push(num_particles_width * cur_y + cur_x - 1);
                                neibor.push_back(sum - 1);
                                ptc_left->c_pos = sum - 1;
                            } else {
                                neibor.push_back(ptc_left->c_pos);
                            }
                        }
                    }

                    if (cur_x < num_particles_width - 1) {
                        Particle *ptc_right = getParticle(cur_x + 1, cur_y);

                        if (ptc_right->isMovable()) {
                            if (!ptc_right->isVisited) {
                                sum++;
                                ptc_right->isVisited = true;
                                connected.push_back(XY(cur_x + 1, cur_y));
                                que.push(num_particles_width * cur_y + cur_x + 1);
                                neibor.push_back(sum - 1);
                                ptc_right->c_pos = sum - 1;
                            } else {
                                neibor.push_back(ptc_right->c_pos);
                            }
                        }
                    }

                    if (cur_y > 0) {
                        Particle *ptc_bottom = getParticle(cur_x, cur_y - 1);

                        if (ptc_bottom->isMovable()) {
                            if (!ptc_bottom->isVisited) {
                                sum++;
                                ptc_bottom->isVisited = true;
                                connected.push_back(XY(cur_x, cur_y - 1));
                                que.push(num_particles_width * (cur_y - 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_bottom->c_pos = sum - 1;
                            } else {
                                neibor.push_back(ptc_bottom->c_pos);
                            }
                        }
                    }

                    if (cur_y < num_particles_height - 1) {
                        Particle *ptc_top = getParticle(cur_x, cur_y + 1);

                        if (ptc_top->isMovable()) {
                            if (!ptc_top->isVisited) {
                                sum++;
                                ptc_top->isVisited = true;
                                connected.push_back(XY(cur_x, cur_y + 1));
                                que.push(num_particles_width * (cur_y + 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_top->c_pos = sum - 1;
                            } else {
                                neibor.push_back(ptc_top->c_pos);
                            }
                        }
                    }
                    neibors.push_back(neibor);
                }

                if (sum > MAX_PARTICLE_FOR_POSTPROCESSIN) {
                    vector<int> edgePoints = findUnmovablePoint(connected);
                    handle_slop_connected(edgePoints, connected, neibors);
                }
            }
        }
    }
}

vector<int> Cloth::findUnmovablePoint(vector<XY> connected) {
    vector<int> edgePoints;

    for (size_t i = 0; i < connected.size(); i++) {
        int x         = connected[i].x;
        int y         = connected[i].y;
        int index     = y * num_particles_width + x;
        Particle *ptc = getParticle(x, y);

        if (x > 0) {
            Particle *ptc_x = getParticle(x - 1, y);

            if (!ptc_x->isMovable()) {
                int index_ref = y * num_particles_width + x - 1;

                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold)) {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (x < num_particles_width - 1) {
            Particle *ptc_x = getParticle(x + 1, y);

            if (!ptc_x->isMovable()) {
                int index_ref = y * num_particles_width + x + 1;

                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold)) {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y > 0) {
            Particle *ptc_y = getParticle(x, y - 1);

            if (!ptc_y->isMovable()) {
                int index_ref = (y - 1) * num_particles_width + x;

                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold)) {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y < num_particles_height - 1) {
            Particle *ptc_y = getParticle(x, y + 1);

            if (!ptc_y->isMovable()) {
                int index_ref = (y + 1) * num_particles_width + x;

                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold)) {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }
    }

    return edgePoints;
}

void Cloth::handle_slop_connected(vector<int> edgePoints, vector<XY> connected, vector<vector<int> > neibors) {
    vector<bool> visited;

    for (size_t i = 0; i < connected.size(); i++) visited.push_back(false);

    queue<int> que;

    for (size_t i = 0; i < edgePoints.size(); i++) {
        que.push(edgePoints[i]);
        visited[edgePoints[i]] = true;
    }

    while (!que.empty()) {
        int index = que.front();
        que.pop();

        int index_center = connected[index].y * num_particles_width + connected[index].x;

        for (size_t i = 0; i < neibors[index].size(); i++) {
            int index_neibor = connected[neibors[index][i]].y * num_particles_width + connected[neibors[index][i]].x;

            if ((fabs(heightvals[index_center] - heightvals[index_neibor]) < smoothThreshold) &&
                (fabs(particles[index_neibor].getPos().f[1] - heightvals[index_neibor]) < heightThreshold)) {
                Vec3 offsetVec = Vec3(0, heightvals[index_neibor] - particles[index_neibor].getPos().f[1], 0);
                particles[index_neibor].offsetPos(offsetVec);
                particles[index_neibor].makeUnmovable();

                if (visited[neibors[index][i]] == false) {
                    que.push(neibors[index][i]);
                    visited[neibors[index][i]] = true;
                }
            }
        }
    }
}

void Cloth::saveToFile(string path) {
    string filepath = "cloth_nodes.txt";

    if (path == "") {
        filepath = "cloth_nodes.txt";
    } else {
        filepath = path;
    }

    ofstream f1(filepath.c_str());

    if (!f1)
        return;

    for (size_t i = 0; i < particles.size(); i++) {
        f1 << fixed << setprecision(8) << particles[i].getPos().f[0] << "	"<< particles[i].getPos().f[2] << "	"<< -particles[i].getPos().f[1] << endl;
    }

    f1.close();
}

void Cloth::saveMovableToFile(string path) {
    string filepath = "cloth_movable.txt";

    if (path == "") {
        filepath = "cloth_movable.txt";
    } else {
        filepath = path;
    }

    ofstream f1(filepath.c_str());

    if (!f1)
        return;

    for (size_t i = 0; i < particles.size(); i++) {
        if (particles[i].isMovable()) {
            f1 << fixed << setprecision(8) << particles[i].getPos().f[0] << "	"
               << particles[i].getPos().f[2] << "	"<< -particles[i].getPos().f[1] << endl;
        }
    }

    f1.close();
}
