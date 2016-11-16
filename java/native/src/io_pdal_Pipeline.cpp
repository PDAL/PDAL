#include <stdio.h>
#include <iostream>
#include <string>

#include "io_pdal_Pipeline.h"
#include "JavaPipeline.hpp"
#include "JavaIterator.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using libpdaljava::PointViewIterator;

using pdal::PointViewSet;
using pdal::PointView;
using pdal::PointViewLess;
using pdal::PointViewPtr;

JNIEXPORT void JNICALL Java_io_pdal_Pipeline_initialise
  (JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(c, "json", "Ljava/lang/String;");
    jstring jstr = (jstring) env->GetObjectField(obj, fid);
    setHandle(env, obj, new Pipeline(std::string(env->GetStringUTFChars(jstr, 0))));
}

JNIEXPORT void JNICALL Java_io_pdal_Pipeline_dispose
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    setHandle<int>(env, obj, 0);
    delete p;
}

JNIEXPORT void JNICALL Java_io_pdal_Pipeline_execute
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    p->execute();
}

JNIEXPORT jstring JNICALL Java_io_pdal_Pipeline_getMetadata
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getMetadata().c_str());
}

JNIEXPORT jstring JNICALL Java_io_pdal_Pipeline_getSchema
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getSchema().c_str());
}

JNIEXPORT jboolean JNICALL Java_io_pdal_Pipeline_validate
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    bool result;
    try
    {
        result = p->validate();
    }
    catch(const pdal::pdal_error& pe)
    {
        std::cerr << "Runtime error: " << pe.what() << std::endl;
        result = false;
    }

    return result;
}

JNIEXPORT void JNICALL Java_io_pdal_Pipeline_setLogLevel
  (JNIEnv *env, jobject obj, jint i)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    p->setLogLevel(i);
}

JNIEXPORT jint JNICALL Java_io_pdal_Pipeline_getLogLevel
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return p->getLogLevel();
}

JNIEXPORT jstring JNICALL Java_io_pdal_Pipeline_getLog
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    return env->NewStringUTF(p->getLog().c_str());
}

JNIEXPORT jobject JNICALL Java_io_pdal_Pipeline_pointViews
  (JNIEnv *env, jobject obj)
{
    Pipeline *p = getHandle<Pipeline>(env, obj);
    PointViewSet pvset = p->getPointViews();

    jclass pviClass = env->FindClass("io/pdal/PointViewIterator");
    jmethodID pviCtor = env->GetMethodID(pviClass, "<init>", "()V");
    jobject pvi = env->NewObject(pviClass, pviCtor);

    PointViewIterator *it = new PointViewIterator(pvset);

    setHandle(env, pvi, it);

    return pvi;
}
