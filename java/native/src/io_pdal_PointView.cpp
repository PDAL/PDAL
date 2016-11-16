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

/// Converts JavaArray of DimTypes (In Java interpretation DimType is a pair of strings)
/// into pdal::DimTypeList (vector of DimTypes), puts dim size into bufSize
/// \param[in] env       JNI environment
/// \param[in] dims      JavaArray of DimTypes
/// \param[in] bufSize   Dims sum size
/// \param[in] dimTypes  Vector of DimTypes
void convertDimTypeJavaArrayToVector(JNIEnv *env, jobjectArray dims, std::size_t *bufSize, pdal::DimTypeList *dimTypes) {
    for (jint i = 0; i < env->GetArrayLength(dims); i++) {
        jobject jDimType = (jobject) env->GetObjectArrayElement(dims, i);
        jclass cDimType = env->GetObjectClass(jDimType);
        jfieldID fid = env->GetFieldID(cDimType, "id", "Ljava/lang/String;");
        jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");
        jfieldID fscale = env->GetFieldID(cDimType, "scale", "D");
        jfieldID foffset = env->GetFieldID(cDimType, "offset", "D");

        jstring jid = (jstring) env->GetObjectField(jDimType, fid);
        jstring jtype = (jstring) env->GetObjectField(jDimType, ftype);
        jdouble jscale = env->GetDoubleField(jDimType, fscale);
        jdouble joffset = env->GetDoubleField(jDimType, foffset);

        Id id = pdal::Dimension::id(std::string(env->GetStringUTFChars(jid, 0)));
        Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));

        *bufSize += pdal::Dimension::size(type);
        dimTypes->insert(dimTypes->begin() + i, pdal::DimType(id, type, jscale, joffset));
    }
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
  (JNIEnv *env, jobject obj, jint mode_flag, jboolean pretty)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    pdal::SpatialReference::WKTModeFlag enumFlag;

    switch(mode_flag)
    {
        case 2:
            enumFlag = (pdal::SpatialReference::WKTModeFlag) mode_flag;
            break;
        default:
            enumFlag = (pdal::SpatialReference::WKTModeFlag) 1;
            break;
    }

    return env->NewStringUTF(pv->spatialReference().getWKT(enumFlag, pretty).c_str());
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPoint
  (JNIEnv *env, jobject obj, jobjectArray dims, jlong idx)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t bufSize = 0;
    pdal::DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &bufSize, &dimTypes);

    char *buf = new char[bufSize];
    pv->getPackedPoint(dimTypes, idx, buf);

    jbyteArray array = env->NewByteArray(bufSize);
    env->SetByteArrayRegion (array, 0, bufSize, reinterpret_cast<jbyte *>(buf));

    return array;
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPoints
  (JNIEnv *env, jobject obj, jobjectArray dims)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t bufSize = 0;
    pdal::DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &bufSize, &dimTypes);

    // reading all points
    bufSize = bufSize * pv->size();
    char *buf = new char[bufSize];

    for (int idx = 0; idx < pv->size(); idx++) {
        pv->getPackedPoint(dimTypes, idx, buf);
    }

    jbyteArray array = env->NewByteArray(bufSize);
    env->SetByteArrayRegion (array, 0, bufSize, reinterpret_cast<jbyte *>(buf));

    return array;
}

JNIEXPORT void JNICALL Java_io_pdal_PointView_dispose
  (JNIEnv *env, jobject obj)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    setHandle<int>(env, obj, 0);
    delete pvrp;
}
