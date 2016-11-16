#include <stdio.h>
#include "io_pdal_PointLayout.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using pdal::PointLayout;
using pdal::DimTypeList;
using pdal::DimType;
using pdal::XForm;

JNIEXPORT jobjectArray JNICALL Java_io_pdal_PointLayout_dimTypes
  (JNIEnv *env, jobject obj)
{
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    DimTypeList dimTypes = pl->dimTypes();

    jclass dtClass = env->FindClass("io/pdal/DimType");
    jmethodID dtCtor = env->GetMethodID(dtClass, "<init>", "(Ljava/lang/String;Ljava/lang/String;DD)V");

    jobjectArray result = env->NewObjectArray(dimTypes.size(), dtClass, NULL);

    for (long i = 0; i < dimTypes.size(); i++)
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
    XForm xform = dt.m_xform;

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

JNIEXPORT jlong JNICALL Java_io_pdal_PointLayout_dimOffset
  (JNIEnv *env, jobject obj, jstring jstr)
{
    std::string fid = std::string(env->GetStringUTFChars(jstr, 0));
    PointLayout *pl = getHandle<PointLayout>(env, obj);

    return pl->dimOffset(pl->findDim(fid));
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
