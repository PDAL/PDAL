/******************************************************************************
* Copyright (c) 2016, hobu Inc.  (info@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

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
using pdal::pdal_error;

JNIEXPORT void JNICALL Java_io_pdal_Pipeline_initialise
  (JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(c, "json", "Ljava/lang/String;");
    jstring jstr = reinterpret_cast<jstring>(env->GetObjectField(obj, fid));
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
    catch(const pdal_error& pe)
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

JNIEXPORT jobject JNICALL Java_io_pdal_Pipeline_getPointViews
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
