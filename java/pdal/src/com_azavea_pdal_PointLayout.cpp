#include <stdio.h>
#include "com_azavea_pdal_PointLayout.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using pdal::PointLayout;
using pdal::DimTypeList;

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointLayout_dimTypes
  (JNIEnv *env, jobject obj)
{
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    DimTypeList dimTypes = pl->dimTypes();

    jclass arrayClass   = env->FindClass("java/util/ArrayList");
    jmethodID arrayCtor = env->GetMethodID(arrayClass, "<init>", "(I)V");
    jmethodID arraySize = env->GetMethodID(arrayClass, "size", "()I");
    jmethodID arrayGet  = env->GetMethodID(arrayClass, "get", "(I)Ljava/lang/Object;");
    jmethodID arrayAdd  = env->GetMethodID(arrayClass, "add", "(Ljava/lang/Object;)Z");

    jobject result = env->NewObject(arrayClass, arrayCtor, dimTypes.size());

    for (auto dt: dimTypes)
    {
        jstring element = env->NewStringUTF(pdal::Dimension::name(dt.m_id).c_str());
        env->CallBooleanMethod(result, arrayAdd, element);
        env->DeleteLocalRef(element);
    }

    return result;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_PointLayout_dispose
  (JNIEnv *env, jobject obj)
{
    PointLayout *pl = getHandle<PointLayout>(env, obj);
    setHandle<int>(env, obj, 0);
    delete pl;
}
