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
#include "io_pdal_PointView.h"
#include "JavaPipeline.hpp"
#include "PointViewRawPtr.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using libpdaljava::PointViewRawPtr;

using pdal::PointView;
using pdal::PointViewPtr;
using pdal::PointLayoutPtr;
using pdal::Dimension::Type;
using pdal::Dimension::Id;
using pdal::PointId;
using pdal::DimTypeList;
using pdal::SpatialReference;
using pdal::DimType;

/// Converts JavaArray of DimTypes (In Java interpretation DimType is a pair of strings)
/// into DimTypeList (vector of DimTypes), puts dim size into bufSize
/// \param[in] env       JNI environment
/// \param[in] dims      JavaArray of DimTypes
/// \param[in] bufSize   Dims sum size
/// \param[in] dimTypes  Vector of DimTypes
void convertDimTypeJavaArrayToVector(JNIEnv *env, jobjectArray dims, std::size_t *pointSize, DimTypeList *dimTypes) {
    for (jint i = 0; i < env->GetArrayLength(dims); i++) {
        jobject jDimType = reinterpret_cast<jobject>(env->GetObjectArrayElement(dims, i));
        jclass cDimType = env->GetObjectClass(jDimType);
        jfieldID fid = env->GetFieldID(cDimType, "id", "Ljava/lang/String;");
        jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");
        jfieldID fscale = env->GetFieldID(cDimType, "scale", "D");
        jfieldID foffset = env->GetFieldID(cDimType, "offset", "D");

        jstring jid = reinterpret_cast<jstring>(env->GetObjectField(jDimType, fid));
        jstring jtype = reinterpret_cast<jstring>(env->GetObjectField(jDimType, ftype));
        jdouble jscale = env->GetDoubleField(jDimType, fscale);
        jdouble joffset = env->GetDoubleField(jDimType, foffset);

        Id id = pdal::Dimension::id(std::string(env->GetStringUTFChars(jid, 0)));
        Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));

        *pointSize += pdal::Dimension::size(type);
        dimTypes->insert(dimTypes->begin() + i, DimType(id, type, jscale, joffset));
    }
}

/// Fill a buffer with point data specified by the dimension list, accounts index
/// Using this functions it is possible to pack all points into one buffer
/// \param[in] pv    PointView pointer.
/// \param[in] dims  List of dimensions/types to retrieve.
/// \param[in] idx   Index of point to get.
/// \param[in] buf   Pointer to buffer to fill.
void appendPackedPoint(PointViewPtr pv, const DimTypeList& dims, PointId idx, std::size_t pointSize, char *buf)
{
    std::size_t from = idx * pointSize;
    if(from >= pv->size() * pointSize) return;
    buf += from;
    pv->getPackedPoint(dims, idx, buf);
}

JNIEXPORT jobject JNICALL Java_io_pdal_PointView_layout
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;
    PointLayoutPtr pl = pv->layout();

    jclass pvlClass = env->FindClass("io/pdal/PointLayout");
    jmethodID pvlCtor = env->GetMethodID(pvlClass, "<init>", "()V");
    jobject pvl = env->NewObject(pvlClass, pvlCtor);

    setHandle(env, pvl, pl);

    return pvl;
}

JNIEXPORT jint JNICALL Java_io_pdal_PointView_size
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;
    return pv->size();
}

JNIEXPORT jboolean JNICALL Java_io_pdal_PointView_empty
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;
    return pv->empty();
}

JNIEXPORT jstring JNICALL Java_io_pdal_PointView_getCrsProj4
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;
    return env->NewStringUTF(pv->spatialReference().getProj4().c_str());
}

JNIEXPORT jstring JNICALL Java_io_pdal_PointView_getCrsWKT
  (JNIEnv *env, jobject obj, jboolean pretty)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    std::string wkt = pv->spatialReference().getWKT();

    if(pretty) wkt = SpatialReference::prettyWkt(wkt);

    return env->NewStringUTF(wkt.c_str());
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPoint
  (JNIEnv *env, jobject obj, jlong idx, jobjectArray dims)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t pointSize = 0;
    DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &pointSize, &dimTypes);

    char *buf = new char[pointSize];

    pv->getPackedPoint(dimTypes, idx, buf);

    jbyteArray array = env->NewByteArray(pointSize);
    env->SetByteArrayRegion (array, 0, pointSize, reinterpret_cast<jbyte *>(buf));

    delete[] buf;

    return array;
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPoints
  (JNIEnv *env, jobject obj, jobjectArray dims)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t pointSize = 0;
    DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &pointSize, &dimTypes);

    // reading all points
    std::size_t bufSize = pointSize * pv->size();
    char *buf = new char[bufSize];

    for (PointId idx = 0; idx < pv->size(); idx++) {
        appendPackedPoint(pv, dimTypes, idx, pointSize, buf);
    }

    jbyteArray array = env->NewByteArray(bufSize);
    env->SetByteArrayRegion (array, 0, bufSize, reinterpret_cast<jbyte *>(buf));

    delete[] buf;

    return array;
}

JNIEXPORT void JNICALL Java_io_pdal_PointView_dispose
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    setHandle<int>(env, obj, 0);
    delete pvrp;
}
