#include <stdio.h>
#include "com_azavea_pdal_Pipeline.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using pdal::PointView;

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointView_points
  (JNIEnv *env, jobject obj)
{
    PointView *p = getHandle<PointView>(env, obj);
    //return;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_PointView_dispose
  (JNIEnv *env, jobject obj)
{
    PointView *p = getHandle<PointView>(env, obj);
    setHandle<int>(env, obj, 0);
    delete p;
}
