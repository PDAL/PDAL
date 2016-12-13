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
#include "io_pdal_PointViewIterator.h"
#include "JavaPipeline.hpp"
#include "JavaIterator.hpp"
#include "PointViewRawPtr.hpp"
#include "Accessors.hpp"

using libpdaljava::PointViewIterator;
using libpdaljava::PointViewRawPtr;

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
