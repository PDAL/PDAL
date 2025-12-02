#include <Eigen/Dense>

#include <pdal/KDIndex.hpp>

namespace pdal
{

using namespace Eigen;

struct NormalProcessor
{

    NormalProcessor() : m_normal(Vector3d::Zero()), m_curvature(0.0)
    {}

    Vector3d m_normal;
    double m_curvature;

    void findNormal(PointView& view, PointIdList neighbors)
    {
        // Check if the covariance matrix is all zeros
        auto B = math::computeCovariance(view, neighbors);
        if (B.isZero())
            return;
    
        SelfAdjointEigenSolver<Matrix3d> solver(B);
        if (solver.info() != Success)
            throw pdal_error("Cannot perform eigen decomposition during normal calculation.");
    
        // The curvature is computed as the ratio of the first (smallest)
        // eigenvalue to the sum of all eigenvalues.
        auto eval = solver.eigenvalues();
        double sum = eval[0] + eval[1] + eval[2];
    
        m_curvature = sum ? std::fabs(eval[0] / sum) : 0;
    
        // The normal is defined by the eigenvector corresponding to the
        // smallest eigenvalue.
        m_normal = solver.eigenvectors().col(0);
    }

    // Compute the normal at a particular point, using points within a radius.
    void findNormal(double x, double y, double z, PointView& v, double radius)
    {
        KD3Index& kdi = v.build3dIndex();
        PointIdList neighbors = kdi.radius(x, y, z, radius);
        if (neighbors.size() < 3)
            return;
        findNormal(v, neighbors);
    }

    // Compute the normal at a particular point, using k-nearest neighbors.
    void findNormal(double x, double y, double z, PointView& v, int knn)
    { findNormal(v, v.build3dIndex().neighbors(x, y, z, knn)); }
    
    // If normals are expected to be upward facing, invert them when the
    // Z component is negative.
    void orientUp()
    {
        if (m_normal[2] < 0)
            m_normal *= -1;
    }
    // Where the dot product of the vector connecting a point with the
    // viewpoint and the normal is negative, flip the normal.
    void orientViewpoint(const Vector3d& vp)
    {
        if (vp.dot(m_normal) < 0)
            m_normal *= -1;
    }
};

} // namespace pdal