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
#include <vector>
#include "io_pdal_PointLayout.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using pdal::PointLayout;
using pdal::DimTypeList;
using pdal::DimType;

JNIEXPORT jobjectArray JNICALL Java_io_pdal_PointLayout_dimTypes
  (JNIEnv *env, jobject obj)
{
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    DimTypeList dimTypes = pl->dimTypes();

    jclass dtClass = env->FindClass("io/pdal/DimType");
    jmethodID dtCtor = env->GetMethodID(dtClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;DD)V");

    jobjectArray result = env->NewObjectArray(dimTypes.size(), dtClass, NULL);

    for (long i = 0; i < static_cast<long>(dimTypes.size()); i++)
    {
        auto dt = dimTypes.at(i);
        jstring id = env->NewStringUTF(pdal::Dimension::name(dt.m_id).c_str());
        jstring type = env->NewStringUTF(pdal::Dimension::interpretationName(dt.m_type).c_str());
        jobject element = env->NewObject(dtClass, dtCtor, id, type, dt.m_xform.m_scale.m_val, dt.m_xform.m_offset.m_val);

        env->SetObjectArrayElement(result, i, element);

        env->DeleteLocalRef(element);
        env->DeleteLocalRef(type);
        env->DeleteLocalRef(id);
    }

    return result;
}

JNIEXPORT jobject JNICALL Java_io_pdal_PointLayout_findDimType
  (JNIEnv *env, jobject obj, jstring jstr)
{
    std::string fid = std::string(env->GetStringUTFChars(jstr, 0));
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    DimType dt = pl->findDimType(fid);
    jstring id = env->NewStringUTF(pdal::Dimension::name(dt.m_id).c_str());
    jstring type = env->NewStringUTF(pdal::Dimension::interpretationName(dt.m_type).c_str());

    jclass dtClass = env->FindClass("io/pdal/DimType");
    jmethodID dtCtor = env->GetMethodID(dtClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;DD)V");
    jobject result = env->NewObject(dtClass, dtCtor, id, type, dt.m_xform.m_scale.m_val, dt.m_xform.m_offset.m_val);

    return result;
}

JNIEXPORT jlong JNICALL Java_io_pdal_PointLayout_dimSize
  (JNIEnv *env, jobject obj, jstring jstr)
{
    std::string fid = std::string(env->GetStringUTFChars(jstr, 0));
    PointLayout *pl = getHandle<PointLayout>(env, obj);

    return pl->dimSize(pl->findDim(fid));
}

JNIEXPORT jlong JNICALL Java_io_pdal_PointLayout_dimPackedOffset
  (JNIEnv *env, jobject obj, jstring jstr)
{
    std::string fid = std::string(env->GetStringUTFChars(jstr, 0));
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    DimType dimType = pl->findDimType(fid);
    DimTypeList dims = pl->dimTypes();

    auto it = std::find_if(dims.begin(), dims.end(), [&dimType](const DimType& dt) {
        return pdal::Dimension::name(dt.m_id) == pdal::Dimension::name(dimType.m_id);
    });
    auto index = std::distance(dims.begin(), it);
    long offset = 0;

    for(int i = 0; i < index; i++)
    {
        offset += pl->dimSize(dims.at(i).m_id);
    }

    return offset;
}

JNIEXPORT jlong JNICALL Java_io_pdal_PointLayout_pointSize
  (JNIEnv *env, jobject obj)
{
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    return pl->pointSize();
}

JNIEXPORT void JNICALL Java_io_pdal_PointLayout_dispose
  (JNIEnv *env, jobject obj)
{
    // A bit unclear why we can't remove this pointer, probably wrapping here makes sense as well
    // PointLayout *pl = getHandle<PointLayout>(env, obj);
    setHandle<int>(env, obj, 0);
    // delete pl;
}
