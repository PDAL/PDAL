#include "JavaPipeline.hpp"

#ifndef _JAVAPOINTVIEW_H_INCLUDED_
#define _JAVAPOINTVIEW_H_INCLUDED_

using pdal::PointViewPtr;

/**
 * PointView wrapper for safer work with PointView shared_pointer as with a common pointer
 */
namespace libpdaljava
{
class PointViewRawPtr
{
public:
    PointViewPtr shared_pointer;

    PointViewRawPtr(PointViewPtr);
    ~PointViewRawPtr();
};
}
#endif
