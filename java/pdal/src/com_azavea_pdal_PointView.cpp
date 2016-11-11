#include <stdio.h>
#include "com_azavea_pdal_Pipeline.h"
#include "JavaPipeline.hpp"
#include "Accessors.h"

using libpdaljava::Pipeline;

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointView_points
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
}
