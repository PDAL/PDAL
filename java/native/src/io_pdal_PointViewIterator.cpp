#include <stdio.h>
#include "io_pdal_PointViewIterator.h"
#include "JavaPipeline.hpp"
#include "JavaIterator.hpp"
#include "PointViewRawPtr.hpp"
#include "Accessors.hpp"

using libpdaljava::PointViewIterator;
using libpdaljava::PointViewRawPtr;

using pdal::PointViewSet;
using pdal::PointView;
using pdal::PointViewPtr;
using pdal::PointViewLess;

JNIEXPORT jboolean JNICALL Java_io_pdal_PointViewIterator_hasNext
  (JNIEnv *env, jobject obj)
{
    PointViewIterator *it = getHandle<PointViewIterator>(env, obj);
    return it->hasNext();
}

JNIEXPORT jobject JNICALL Java_io_pdal_PointViewIterator_next
  (JNIEnv *env, jobject obj)
{
    PointViewIterator *it = getHandle<PointViewIterator>(env, obj);

    PointViewPtr pvptr = it->next();

    jclass jpvClass = env->FindClass("io/pdal/PointView");
    jmethodID jpvCtor = env->GetMethodID(jpvClass, "<init>", "()V");
    jobject jpv = env->NewObject(jpvClass, jpvCtor);

    PointViewRawPtr *pvrp = new PointViewRawPtr(pvptr);

    setHandle(env, jpv, pvrp);

    return jpv;
}

JNIEXPORT void JNICALL Java_io_pdal_PointViewIterator_dispose
  (JNIEnv *env, jobject obj)
{
    PointViewIterator *it = getHandle<PointViewIterator>(env, obj);
    setHandle<int>(env, obj, 0);
    delete it;
}
