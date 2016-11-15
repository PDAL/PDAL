#include "PointViewRawPtr.hpp"

using pdal::PointViewPtr;

namespace libpdaljava
{
PointViewRawPtr::PointViewRawPtr(PointViewPtr p)
    : shared_pointer{p}
{ }

PointViewRawPtr::~PointViewRawPtr()
{ }
}
