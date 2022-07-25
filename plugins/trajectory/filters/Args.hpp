//
// (c) 2022 SRI International
//
#pragma once

#include <iostream>

namespace pdal
{
namespace trajectory
{

struct Args
{
    double dtr;
    double dts;
    double minsep;
    double tblock;
    double tout;

    // These aren't currently real args and need initialization here.
    double flipScanAngle { 1.0 };
    double dr { 0.01 };
    double accelweight { 0.5 };
    double clampweight { 0.0001 };
    double straddleweight { 0.0001 };
    int estn { 20 };
    int niter { 50 };
    double dang { 1 };
    double fixedpitch { std::numeric_limits<double>::quiet_NaN() };
    double multiweight { 5 };
    double pitchweight { 1 };
    double scanweight { .005 };
    double scanweightest { .01 };
    double attaccelweight { .01 };
    double attclampweight { .0001 };
    double extrapitchclamp { 1 };

    void Dump() const
    {
        std::cerr << "dtr = " << dtr << "!\n";
        std::cerr << "dts = " << dts << "!\n";
        std::cerr << "tblock = " << tblock << "!\n";
        std::cerr << "tout = " << tout << "!\n";
        std::cerr << "dr = " << dr << "!\n";
        std::cerr << "minsep = " << minsep << "!\n";
        std::cerr << "accelweight = " << accelweight << "!\n";
        std::cerr << "clampweight = " << clampweight << "!\n";
        std::cerr << "straddleweight = " << straddleweight << "!\n";
        std::cerr << "estn = " << estn << "!\n";
        std::cerr << "niter = " << niter << "!\n";
        std::cerr << "flipscanang = " << flipScanAngle << "!\n";
        std::cerr << "dang = " << dang << "!\n";
        std::cerr << "pitchweight = " << pitchweight << "!\n";
        std::cerr << "scanweightest = " << scanweightest << "!\n";
        std::cerr << "fixedpitch = " << fixedpitch << "!\n";
        std::cerr << "multiweight = " << multiweight << "!\n";
        std::cerr << "scanweight = " << scanweight << "!\n";
        std::cerr << "attaccelweight = " << attaccelweight << "!\n";
        std::cerr << "attclampweight = " << attclampweight << "!\n";
        std::cerr << "extrapitchclamp = " << extrapitchclamp << "!\n";
    }
};

} // namespace trajectory
} // namespace pdal
