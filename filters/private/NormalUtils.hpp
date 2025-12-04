#pragma once

#include <Eigen/Dense>

#include <pdal/KDIndex.hpp>

#include <pdal/private/MathUtils.hpp>

namespace pdal
{

using namespace Eigen;

struct NormalResult
{
    Vector3d normal { Vector3d::Zero() };
    double curvature { 0.0 };
    std::string msg;

    // If normals are expected to be upward facing, invert them when the
    // Z component is negative.
    void orientUp()
    {
        if (normal[2] < 0)
            normal = -normal;
    }
    // Where the dot product of the vector connecting a point with the
    // viewpoint and the normal is negative, flip the normal.
    void orientViewpoint(const Vector3d& vp)
    {
        if (vp.dot(normal) < 0)
            normal = -normal;
    }
};

namespace math
{

inline NormalResult findNormal(PointView& view, PointIdList neighbors)
{
    NormalResult result;
    if (neighbors.size() < 3)
    {
        result.msg = "Not enough neighbors to compute normal.";
        return result;
    }

    // Check if the covariance matrix is all zeros
    auto B = math::computeCovariance(view, neighbors);
    if (B.isZero())
    {
        result.msg = "Covariance matrix is all zeros. This suggests a large "
            "number of redundant points.";
        return result;
    }

    SelfAdjointEigenSolver<Matrix3d> solver(B);
    if (solver.info() != Success)
    {
        result.msg = "Cannot perform eigen decomposition during normal calculation.";
        return result;
    }

    // The curvature is computed as the ratio of the first (smallest)
    // eigenvalue to the sum of all eigenvalues.
    auto eval = solver.eigenvalues();
    double sum = eval[0] + eval[1] + eval[2];

    result.curvature = sum ? std::fabs(eval[0] / sum) : 0;

    // The normal is defined by the eigenvector corresponding to the
    // smallest eigenvalue.
    result.normal = solver.eigenvectors().col(0);

    return result;
}
// Compute the normal at a particular point, using neighbors within a radius.
inline NormalResult findNormal(double x, double y, double z, PointView& v, double radius)
{ return findNormal(v, v.build3dIndex().radius(x, y, z, radius)); }

// Compute the normal at a particular point, using k-nearest neighbors.
inline NormalResult findNormal(double x, double y, double z, PointView& v, int knn)
{ return findNormal(v, v.build3dIndex().neighbors(x, y, z, knn)); }

} // namespace math
} // namespace pdal
