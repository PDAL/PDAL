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

const int endian_check = 1;
#define is_littleendian() ((*(char*)&endian_check) != 0)

/// Converts JavaArray of DimTypes (In Java interpretation DimType is a pair of strings)
/// into pdal::DimTypeList (vector of DimTypes), puts dim size into bufSize
/// \param[in] env       JNI environment
/// \param[in] dims      JavaArray of DimTypes
/// \param[in] bufSize   Dims sum size
/// \param[in] dimTypes  Vector of DimTypes
void convertDimTypeJavaArrayToVector(JNIEnv *env, jobjectArray dims, std::size_t *pointSize, pdal::DimTypeList *dimTypes) {
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

        *pointSize += pdal::Dimension::size(type);
        dimTypes->insert(dimTypes->begin() + i, pdal::DimType(id, type, jscale, joffset));
    }
}

/// Fill a buffer with point data specified by the dimension list.
/// \param[in] dims  List of dimensions/types to retrieve.
/// \param[in] idx   Index of point to get.
/// \param[in] buf   Pointer to buffer to fill.
void getPackedPoint(PointViewPtr pv, const pdal::DimTypeList& dims, pdal::PointId idx, char *buf)
{
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        const pdal::Dimension::Detail *d = pv->layout()->dimDetail(di->m_id);
        char *chunk = new char[pdal::Dimension::size(di->m_type)];
        pv->getField(chunk, di->m_id, di->m_type, idx);
        // JVM native endian conversion
        if(is_littleendian())
        {
            std::reverse(chunk, chunk + pdal::Dimension::size(di->m_type));
        }
        memcpy(buf, chunk, pdal::Dimension::size(di->m_type));
        buf += pdal::Dimension::size(di->m_type);
        delete[] chunk;

    }
}

/// Fill a buffer with point data specified by the dimension list, accounts index
/// Using this functions it is possible to pack all points into one buffer
/// \param[in] pv    pdal::PointView pointer.
/// \param[in] dims  List of dimensions/types to retrieve.
/// \param[in] idx   Index of point to get.
/// \param[in] buf   Pointer to buffer to fill.
void appendPackedPoint(PointViewPtr pv, const pdal::DimTypeList& dims, pdal::PointId idx, std::size_t pointSize, char *buf)
{
    std::size_t from = idx * pointSize;
    if(from >= pv->size() * pointSize)
    {
        return;
    }

    buf += from;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        char *chunk = new char[pdal::Dimension::size(di->m_type)];
        pv->getField(chunk, di->m_id, di->m_type, idx);
        // JVM native endian conversion
        if(is_littleendian())
        {
            std::reverse(chunk, chunk + pdal::Dimension::size(di->m_type));
        }
        memcpy(buf, chunk, pdal::Dimension::size(di->m_type));
        buf += pdal::Dimension::size(di->m_type);
        delete[] chunk;
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
    if(mode_flag > 2 && mode_flag < 1)
    {
        enumFlag = (pdal::SpatialReference::WKTModeFlag) 1;
    }
    else
    {
        enumFlag = (pdal::SpatialReference::WKTModeFlag) mode_flag;
    }

    return env->NewStringUTF(pv->spatialReference().getWKT(enumFlag, pretty).c_str());
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPointBytes
  (JNIEnv *env, jobject obj, jlong idx, jobjectArray dims)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t pointSize = 0;
    pdal::DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &pointSize, &dimTypes);

    char *buf = new char[pointSize];
    getPackedPoint(pv, dimTypes, idx, buf);

    jbyteArray array = env->NewByteArray(pointSize);
    env->SetByteArrayRegion (array, 0, pointSize, reinterpret_cast<jbyte *>(buf));

    delete[] buf;

    return array;
}

JNIEXPORT jbyteArray JNICALL Java_io_pdal_PointView_getPackedPointsBytes
  (JNIEnv *env, jobject obj, jobjectArray dims)
{
    PointViewRawPtr *pvrp = getHandle<PointViewRawPtr>(env, obj);
    PointViewPtr pv = pvrp->shared_pointer;

    PointLayoutPtr pl = pv->layout();

    // we need to calculate buffer size
    std::size_t pointSize = 0;
    pdal::DimTypeList dimTypes;

    // calculate result buffer length (for one point) and get dimTypes
    convertDimTypeJavaArrayToVector(env, dims, &pointSize, &dimTypes);

    // reading all points
    std::size_t bufSize = pointSize * pv->size();
    char *buf = new char[bufSize];

    for (int idx = 0; idx < pv->size(); idx++) {
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
