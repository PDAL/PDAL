#include <stdio.h>
#include "com_azavea_pdal_PointView.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using pdal::PointView;
using pdal::PointLayoutPtr;
using pdal::Dimension::Type;
using pdal::Dimension::Id;
using pdal::PointId;

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointView_layout
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    PointLayoutPtr pl = pv->layout();

    jclass pvlClass = env->FindClass("com/azavea/pdal/PointLayout");
    jmethodID pvlCtor = env->GetMethodID(pvlClass, "<init>", "()V");
    jobject pvl = env->NewObject(pvlClass, pvlCtor);

    setHandle(env, pvl, pl);

    return pvl;
}

JNIEXPORT jint JNICALL Java_com_azavea_pdal_PointView_size
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    return pv->size();
}

JNIEXPORT jboolean JNICALL Java_com_azavea_pdal_PointView_empty
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    return pv->empty();
}

JNIEXPORT jbyteArray JNICALL Java_com_azavea_pdal_PointView_getPackedPoint
  (JNIEnv *env, jobject obj, jobjectArray dims, jlong idx)
{
    PointView *pv = getHandle<PointView>(env, obj);

    PointLayoutPtr pl = pv->layout();

    jint len = env->GetArrayLength(dims);
    // not the best logic to allocate only necessary memory amount
    std::size_t bufSize = pl->pointSize();
    if(pl->dimTypes().size() != len)
    {
        bufSize = 0;
        for (jint i = 0; i < len; i++) {
            jobject jDimType = (jobject) env->GetObjectArrayElement(dims, i);
            jclass cDimType = env->GetObjectClass(jDimType);
            jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");

            jstring jtype = (jstring) env->GetObjectField(jDimType, ftype);
            Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));
            bufSize += pdal::Dimension::size(type);
        }
    }

    char *buf = new char[bufSize];

    for (jint i = 0; i < len; i++) {
        jobject jDimType = (jobject) env->GetObjectArrayElement(dims, i);
        jclass cDimType = env->GetObjectClass(jDimType);
        jfieldID fid = env->GetFieldID(cDimType, "id", "Ljava/lang/String;");
        jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");

        jstring jid = (jstring) env->GetObjectField(jDimType, fid);
        jstring jtype = (jstring) env->GetObjectField(jDimType, ftype);

        Id id = pdal::Dimension::id(std::string(env->GetStringUTFChars(jid, 0)));
        Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));

        pv->getField(buf, id, type, idx);
        buf += pdal::Dimension::size(type);
    }

    jbyteArray array = env->NewByteArray(bufSize);
    env->SetByteArrayRegion (array, 0, bufSize, reinterpret_cast<jbyte *>(buf));

    return array;
}

JNIEXPORT jbyteArray JNICALL Java_com_azavea_pdal_PointView_getPackedPoints
  (JNIEnv *env, jobject obj, jobjectArray dims)
{
    PointView *pv = getHandle<PointView>(env, obj);

    PointLayoutPtr pl = pv->layout();

    jint len = env->GetArrayLength(dims);
    // not the best logic to allocate only necessary memory amount
    std::size_t bufSize = pl->pointSize();
    if(pl->dimTypes().size() != len)
    {
        bufSize = 0;
        for (jint i = 0; i < len; i++) {
            jobject jDimType = (jobject) env->GetObjectArrayElement(dims, i);
            jclass cDimType = env->GetObjectClass(jDimType);
            jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");

            jstring jtype = (jstring) env->GetObjectField(jDimType, ftype);
            Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));
            bufSize += pdal::Dimension::size(type);
        }
    }

    bufSize = bufSize * pv->size();
    char *buf = new char[bufSize];

    for (int idx = 0; idx < pv->size(); idx++) {
        for (jint i = 0; i < len; i++) {
            jobject jDimType = (jobject) env->GetObjectArrayElement(dims, i);
            jclass cDimType = env->GetObjectClass(jDimType);
            jfieldID fid = env->GetFieldID(cDimType, "id", "Ljava/lang/String;");
            jfieldID ftype = env->GetFieldID(cDimType, "type", "Ljava/lang/String;");

            jstring jid = (jstring) env->GetObjectField(jDimType, fid);
            jstring jtype = (jstring) env->GetObjectField(jDimType, ftype);

            Id id = pdal::Dimension::id(std::string(env->GetStringUTFChars(jid, 0)));
            Type type = pdal::Dimension::type(std::string(env->GetStringUTFChars(jtype, 0)));

            pv->getField(buf, id, type, idx);
            buf += pdal::Dimension::size(type);
        }
    }

    jbyteArray array = env->NewByteArray(bufSize);
    env->SetByteArrayRegion (array, 0, bufSize, reinterpret_cast<jbyte *>(buf));

    return array;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_PointView_dispose
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    setHandle<int>(env, obj, 0);
    delete pv;
}
