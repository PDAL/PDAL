#include "SplineFit.hpp"

#include <cmath>

namespace pdal
{
namespace trajectory
{

template <int N>
Eigen::Matrix<double, N, 1> SplineFit<N>::position(double t) const
{
    auto tcomb = tconvert(t);
    int i = tcomb.first;
    double tf = tcomb.second;

    assert(i < num);

    datum p;
    for (int j = 0; j < N; ++j)
        p(j) = SplineFitScalar::EndPointCubic(r[i  ](j), v[i  ](j),
                                              r[i+1](j), v[i+1](j),
                                              tf);
    return p;
}

template <int N>
Eigen::Matrix<double, N, 1> SplineFit<N>::position(double t, SplineFit<N>::datum& velocity) const
{
    auto tcomb = tconvert(t);
    int i = tcomb.first;
    double tf = tcomb.second;

    assert(i < num);

    datum p;
    for (int j = 0; j < N; ++j)
        p(j) = SplineFitScalar::EndPointCubic(r[i  ](j), v[i  ](j),
                                              r[i+1](j), v[i+1](j),
                                              tf, velocity.data() + j);
    velocity /= tblock;
    return p;
}

template <int N>
Eigen::Matrix<double, N, 1> SplineFit<N>::position(double t, SplineFit<N>::datum& velocity,
        SplineFit<N>::datum& acceleration) const
{
    auto tcomb = tconvert(t);
    int i = tcomb.first;
    double tf = tcomb.second;

    assert (i < num);

    datum p;
    for (int j = 0; j < N; ++j)
        p(j) = SplineFitScalar::EndPointCubic(r[i  ](j), v[i  ](j),
                                              r[i+1](j), v[i+1](j),
                                              tf,
                                              velocity.data() + j,
                                              acceleration.data() + j);
    velocity /= tblock;
    acceleration /= tblock*tblock;
    return p;
}

template <int N>
bool SplineFit<N>::fillmissing(bool linearfit)
{
    for (int ii = 0; ii <= num; ++ii)
    {
        // Process in order num, 0, 1, ... num-1, so that all interior cases can
        // be done by interpolation
        int i = (ii == 0 ? num : ii - 1);
        if (!missing[i])
            continue;

        int ia;
        int ib;
        int k = 0;
        if (i == num)
        {
            for (int j = num - 1; j >= 0; --j)
            {
                if (!missing[j])
                {
                    if (k == 0)
                        ib = j;
                    else if (k == 1)
                        ia = j;
                    ++k;
                    if (k == 2)
                        break;
                }
            }
        }
        else if (i == 0)
        {
            for (int j = 1; j <= num; ++j)
            {
                if (!missing[j])
                {
                    if (k == 0)
                        ia = j;
                    else if (k == 1)
                        ib = j;
                    ++k;
                    if (k == 2) break;
                }
            }
        }
        else {
            for (int j = i - 1; j >= 0; --j)
            {
                if (!missing[j] || j == 0)
                {
                    ia = j;
                    ++k;
                    break;
                }
            }
            for (int j = i + 1; j <= num; ++j)
            {
                if (!missing[j] || j == num)
                {
                    ib = j;
                    ++k;
                    break;
                }
            }
        }
        if (k != 2)
            return false;

        // Situation is Valid r/v data at ia and ib and need to interpolation r/v
        // data at i.  New temp tblock is ib-ia (and v's need to be scaled /
        // descaled by ib-ia).  Scaled time for i is (i-(ia+ib)/2)/(ib-ia).
        double tblockn = ib - ia;
        if (linearfit)
        {
            v[i] = (r[ib] - r[ia]) / tblockn;
            r[i] = r[ia] + v[i] * (i - ia);
        }
        else
        {
            double tn = (i - (ia + ib) / 2.0) / tblockn;
            datum vn, rn;
            for (int j = 0; j < N; ++j)
                rn(j) = SplineFitScalar::EndPointCubic(r[ia](j), v[ia](j) * tblockn,
                        r[ib](j), v[ib](j) * tblockn,
                        tn, vn.data() + j);
            r[i] = rn;
            v[i] = vn / tblockn;
        }
    }
    return true;
}
template class SplineFit<3>;
template class SplineFit<2>;

} // namespace trajectory
} // namespace pdal
