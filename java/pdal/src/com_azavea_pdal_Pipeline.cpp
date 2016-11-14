#include <stdio.h>
#include <iostream>
#include <string>

#include "com_azavea_pdal_Pipeline.h"
#include "JavaPipeline.hpp"
#include "JavaIterator.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using libpdaljava::PointViewIterator;

using pdal::PointViewSet;
using pdal::PointView;
using pdal::PointViewLess;
using pdal::PointViewPtr;

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

JNIEXPORT jstring JNICALL Java_com_azavea_pdal_Pipeline_getMetadata
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getMetadata().c_str());
}

JNIEXPORT jstring JNICALL Java_com_azavea_pdal_Pipeline_getSchema
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getSchema().c_str());
}

JNIEXPORT jboolean JNICALL Java_com_azavea_pdal_Pipeline_validate
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return p->validate();
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_Pipeline_setLogLevel
  (JNIEnv *env, jobject obj, jint i)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    p->setLogLevel(i);
}

JNIEXPORT jint JNICALL Java_com_azavea_pdal_Pipeline_getLogLevel
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return p->getLogLevel();
}

JNIEXPORT jstring JNICALL Java_com_azavea_pdal_Pipeline_getLog
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getLog().c_str());
}

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_Pipeline_pointViews__
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    PointViewSet pvset = p->getPointViews();

    jclass pviClass = env->FindClass("com/azavea/pdal/PointViewIterator");
    jmethodID pviCtor = env->GetMethodID(pviClass, "<init>", "()V");
    jobject pvi = env->NewObject(pviClass, pviCtor);

    PointViewIterator *it = new PointViewIterator(pvset);

    setHandle(env, pvi, it);

    return pvi;
}

JNIEXPORT jint JNICALL Java_com_azavea_pdal_Pipeline_test
  (JNIEnv *env, jobject obj)
{
    std::cout << "Hello from C!" << std::endl;
    std::cout << getJson(env, obj) << std::endl;
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return 22;
}