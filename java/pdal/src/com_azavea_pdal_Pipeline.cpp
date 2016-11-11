#include <stdio.h>
#include "com_azavea_pdal_Pipeline.h"
#include "Pipeline.hpp"
#include "accessors.h"

using libpdaljava::Pipeline;
using pdal::PointViewSet;

JNIEXPORT void JNICALL Java_com_azavea_pdal_Pipeline_initialise
  (JNIEnv *env, jobject obj)
{
    setHandle(env, obj, new Pipeline(getJson(env, obj)));
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_Pipeline_dispose
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    setHandle<int>(env, obj, 0);
    delete p;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_Pipeline_execute
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    p->execute();
}

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_Pipeline_pointViews__
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    PointViewSet ps = p->getPointViews();


}

JNIEXPORT jint JNICALL Java_com_azavea_pdal_Pipeline_test
  (JNIEnv *env, jobject obj)
{
    std::cout << "Hello from C!" << std::endl;
    std::cout << getJson(env, obj) << std::endl;
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return p->test();
}